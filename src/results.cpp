/*
 * File:   bunch_tester.cpp
 * Author: daniele
 *
 * Created on 27 novembre 2014, 22.17
 */

#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boldlib.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/intersections.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/centroid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>


#include <sstream>
#include <string>
#include <fstream>

#include <tools.h>
#include <opencv2/core/mat.hpp>

#include "Extractor.h"
#include "Bold3DExtractor.h"
#include "IrosDataset.h"
#include "extractors/extrators_utils.h"
#include "Descriptor.h"
#include "Bold3DDescriptorMultiBunch.h"
#include "Detector.h"
#include "Bold3DMDetector.h"
#include "Bold3DRDetector.h"
#include "BoldDetector.h"
#include "Parameters.h"
#include <boost/filesystem.hpp>
#include <omp.h>

using namespace std;
using namespace BoldLib;




visy::Parameters* parameters;

/*
 *
 */
int
main (int argc, char** argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  /** PARAMETERS */
  parameters = new visy::Parameters(argc, argv);
  parameters->putFloat("gc_th");
  parameters->putFloat("gc_size");
  parameters->putString("detector");
  parameters->putString("model");
  parameters->putString("sizes");
  parameters->putString("nbins");
  parameters->putInt("maxgc");
  parameters->putInt("set");
  parameters->putInt("scene");
  parameters->putInt("nbin");

  visy::dataset::IrosDataset::init();
  visy::dataset::IrosDataset dataset;

  /** SIZES*/
  std::vector<float> sizes = visy::Parameters::parseFloatArray(parameters->getString("sizes"));
  int nbin = parameters->getInt("nbin");

  /** DETECTOR*/
  visy::detectors::Detector * detector;

  if (parameters->getString("detector") == "BOLD3DM")
  {
    detector = new visy::detectors::Bold3DMDetector(sizes, nbin);
  }
  else if (parameters->getString("detector") == "BOLD3DR")
  {
    detector = new visy::detectors::Bold3DRDetector(sizes, nbin);
  }
  else if (parameters->getString("detector") == "BOLD")
  {
    detector = new visy::detectors::BoldDetector(sizes);
  }

  std::cout << "USING DETECTOR: "<<detector->buildName()<<std::endl;


  /** MODELS */
  for (int nm = 0; nm < visy::dataset::IrosDataset::models->size(); nm++)
  {

    /**LOAD MODEL */
    visy::dataset::Model model = visy::dataset::IrosDataset::models->at(nm);
    std::vector<visy::extractors::KeyPoint3D> model_keypoints;
    cv::Mat model_descriptor;
    pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr model_full_cloud(new pcl::PointCloud<PointType>());
    cv::Mat model_rgb;
    Eigen::Matrix4f model_pose;

    if (dataset.loadDescription(model.name, model_keypoints, model_descriptor, model_pose, detector->buildName()))
    {
      visy::dataset::IrosDataset::loadModel(model.name, 0, model_full_cloud, model_cloud, model_rgb, model_pose);
    }
    else
    {
      dataset.fetchFullModel(model.name, model.n_views, model_keypoints, model_descriptor, model_cloud, model_pose, detector);
    }


    for (int si = 0; si < visy::dataset::IrosDataset::scenes->size(); si++)
    {
      visy::dataset::SetScene setScene = visy::dataset::IrosDataset::scenes->at(si);

      for (int ss = 0; ss <= setScene.scene_number; ss++)
      {
        std::cout << model.name << " -> Set:" << setScene.set_number << " Scene:" << ss << std::endl;
        /** LOAD SCENE */
        std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
        cv::Mat scene_descriptor;
        cv::Mat scene_rgb, scene_rgb_full;
        pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());

        visy::dataset::IrosDataset::loadScene(setScene.set_number, ss, scene_cloud, scene_rgb);

        detector->detect(scene_rgb, scene_cloud, scene_keypoints, scene_descriptor);

        /* MATCHES */
        std::vector<cv::DMatch> matches;
        std::vector<cv::DMatch> good_matches;
        visy::tools::matchKeypointsBrute(matches, good_matches, model_descriptor, scene_descriptor, visy::tools::VISY_TOOLS_MATCHING_FULL);

        /* KEYLINES CONSENSUS SET*/
        std::vector<visy::extractors::KeyPoint3D> matched_model_keypoints;
        std::vector<visy::extractors::KeyPoint3D> matched_scene_keypoints;
        std::vector<int> matched_model_keypoints_indices;
        std::vector<int> matched_scene_keypoints_indices;
        pcl::CorrespondencesPtr model_scene_corrs;
        visy::extractors::utils::keypointsConsensusSet(
                model_keypoints,
                scene_keypoints,
                good_matches,
                matched_model_keypoints,
                matched_scene_keypoints,
                matched_model_keypoints_indices,
                matched_scene_keypoints_indices,
                model_scene_corrs);

        int min_gc = 3;
        int max_gc = parameters->getInt("maxgc");

        for (int gc_th = min_gc; gc_th <= max_gc; gc_th++)
        {
          std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
          std::vector < pcl::Correspondences > clustered_corrs;
          visy::extractors::utils::keypointsGeometricConsistencyGrouping(parameters->getFloat("gc_size"), gc_th,
                  matched_model_keypoints,
                  matched_scene_keypoints,
                  model_scene_corrs,
                  rototranslations,
                  clustered_corrs);



          std::cout << "GC: " << gc_th << " FOUND:" << rototranslations.size() << std::endl;

        }
      }
    }



  }


  return 1;
}
