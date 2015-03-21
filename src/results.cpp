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
#include "Bold3DM2Detector.h"
#include "Bold3DR2Detector.h"
#include "Bold3DM2MultiDetector.h"
#include "detectors/detectors_utils.h"

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
  parameters->putFloat("maxgc");
  parameters->putFloat("gc_size");
  parameters->putString("detector");
  parameters->putString("sizes");
  parameters->putInt("nbin");
  parameters->putInt("occlusion");


  int use_occlusion = false;

  int min_gc = 3;
  int max_gc = parameters->getFloat("maxgc") > 0 ? parameters->getFloat("maxgc") : 40;

  visy::dataset::IrosDataset::init();
  visy::dataset::IrosDataset dataset;

  std::vector<float> sizes = visy::Parameters::parseFloatArray(parameters->getString("sizes"));


  /** DETECTOR CREATION */
  visy::detectors::Detector * detector;
  detector = visy::detectors::utils::buildDetectorFromString(parameters->getString("detector"),parameters);



  /** RESULTS SETS*/
  int gcs = max_gc + 1;
  std::vector<visy::dataset::PrecisionRecallRow> precisions(gcs);

  /** TEST NAME*/
  std::stringstream ss;
  ss << "test_" << parameters->getFloat("gc_size") << "_" << detector->buildName();

  std::string test_name = ss.str();


  std::cout << "Start Test: " << test_name << std::endl;

  for (int model_index = 0; model_index < visy::dataset::IrosDataset::models->size(); model_index++)
  {
    /**MODEL */
    visy::dataset::Model model = visy::dataset::IrosDataset::models->at(model_index);
    std::vector<visy::extractors::KeyPoint3D> model_keypoints;
    cv::Mat model_descriptor;
    pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
    cv::Mat model_rgb;
    Eigen::Matrix4f model_pose;

    std::cout << "Loading model: " << model.name << std::endl;
    dataset.fetchFullModel(model.name, model.n_views, model_keypoints, model_descriptor, model_cloud, model_pose, detector);
    std::cout << "Loaded model: " << model.name << " keypoints:" << model_keypoints.size() << std::endl;

    for (int set_index = 0; set_index < visy::dataset::IrosDataset::scenes->size(); set_index++)
    {
      /* LOAD SET */
      visy::dataset::SetScene set = visy::dataset::IrosDataset::scenes->at(set_index);


      for (int scene_index = 0; scene_index <= set.scene_number; scene_index++)
      {
        /**LOAD SCENE */
        std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
        cv::Mat scene_descriptor;
        cv::Mat scene_rgb, scene_rgb_full;
        pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());
        std::vector<visy::dataset::Annotation> scene_annotations;

        visy::dataset::IrosDataset::loadScene(set.set_number, scene_index, scene_cloud, scene_rgb);
        visy::dataset::IrosDataset::loadAnnotiationsFromSceneFile(model.name, set.set_number, scene_index, scene_annotations);

        /** DETECT SCENE*/
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


        cv::Mat precisionMat = cv::Mat(gcs, 6, CV_32F, float(0));


        for (int gc_th = min_gc; gc_th <= max_gc; gc_th++)
        {

          /** MODEL-SCENE MATCH*/
          std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
          std::vector < pcl::Correspondences > clustered_corrs;
          visy::extractors::utils::keypointsGeometricConsistencyGrouping(parameters->getFloat("gc_size"), gc_th,
                  matched_model_keypoints,
                  matched_scene_keypoints,
                  model_scene_corrs,
                  rototranslations,
                  clustered_corrs);

          int N = rototranslations.size();
          int P = visy::dataset::IrosDataset::checkInstancesNumber(model.name, scene_annotations);
          int TP = std::min((int) N, P);
          int FP = N - P > 0 ? N - P : 0;
          int FN = P - TP > 0 ? P - TP : 0;
          int TN = (N <= 0 && P <= 0) ? 1 : 0;


          std::cout << model.name << " -> " << set.set_number << "," << scene_index << " gc:" << gc_th << " ";
          std::cout << " N: " << N;
          std::cout << " P: " << P;
          std::cout << " TP: " << TP;
          std::cout << " FP: " << FP;
          std::cout << " FN: " << FN;
          std::cout << " TN: " << TN;
          std::cout << std::endl;

          precisionMat.at<float>(gc_th, 0) = precisionMat.at<float>(gc_th, 0) + P;
          precisionMat.at<float>(gc_th, 1) = precisionMat.at<float>(gc_th, 1) + N;
          precisionMat.at<float>(gc_th, 2) = precisionMat.at<float>(gc_th, 2) + TP;
          precisionMat.at<float>(gc_th, 3) = precisionMat.at<float>(gc_th, 3) + FP;
          precisionMat.at<float>(gc_th, 4) = precisionMat.at<float>(gc_th, 4) + TN;
          precisionMat.at<float>(gc_th, 5) = precisionMat.at<float>(gc_th, 5) + FN;


        }
        dataset.savePrecisionMat(test_name, precisionMat);
        
        scene_keypoints.clear();
        scene_annotations.clear();
      }
    }
  }


  //  std::vector<visy::extractors::KeyPoint3D> model_keypoints;
  //  cv::Mat model_descriptor;
  //  pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
  //  pcl::PointCloud<PointType>::Ptr model_full_cloud(new pcl::PointCloud<PointType>());
  //  cv::Mat model_rgb;
  //  Eigen::Matrix4f model_pose;
  //
  //  if (dataset.loadDescription(model.name, model_keypoints, model_descriptor, model_pose, detector->buildName()))
  //  {
  //    visy::dataset::IrosDataset::loadModel(model.name, 0, model_full_cloud, model_cloud, model_rgb, model_pose);
  //  }
  //  else
  //  {
  //    dataset.fetchFullModel(model.name, model.n_views, model_keypoints, model_descriptor, model_cloud, model_pose, detector);
  //
  //  }


  //  std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
  //  cv::Mat scene_descriptor;
  //  cv::Mat scene_rgb, scene_rgb_full;
  //  pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());
  //
  //  visy::dataset::IrosDataset::loadScene(parameters->getInt("set"), parameters->getInt("scene"), scene_cloud, scene_rgb);
  //
  //  detector->detect(scene_rgb, scene_cloud, scene_keypoints, scene_descriptor);
  //
  //
  //
  //
  //  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  //
  //  visy::extractors::utils::modelSceneMatch(
  //          model_keypoints,
  //          scene_keypoints,
  //          model_descriptor,
  //          scene_descriptor,
  //          rototranslations,
  //          parameters->getFloat("gc_size"),
  //          parameters->getFloat("gc_th")
  //          );
  //
  //
  //
  //
  //
  //  std::cout << "FOUND:" << rototranslations.size() << std::endl;
  //
  //
  //  for (int i = 0; i < rototranslations.size(); i++)
  //  {
  //    pcl::PointCloud<PointType>::Ptr model_projection(new pcl::PointCloud<PointType>());
  //    pcl::transformPointCloud(*model_cloud, *model_projection, rototranslations[i]);
  //    std::stringstream ss;
  //    ss << "Instance_" << i << "_cloud";
  //    visy::tools::displayCloud(*viewer, model_projection, 0, 255, 0, 3.0f, ss.str());
  //  }
  //
  //
  //
  //  viewer->addPointCloud(scene_cloud, "scene");
  //
  //
  //  //  visy::extractors::KeyPoint3D::draw3DKeyPoints3D(*viewer, model_keypoints, cv::Scalar(0, 255, 0), "ciao", true);
  //
  //  //  for (int i = 0; i < rototranslations.size(); i++)
  //  //  {
  //  //    pcl::PointCloud<PointType>::Ptr model_projection(new pcl::PointCloud<PointType>());
  //  //    pcl::transformPointCloud(*model_cloud, *model_projection, rototranslations[i]);
  //  //    std::stringstream ss;
  //  //    ss << "Instance_" << i << "_cloud";
  //  //    visy::tools::displayCloud(*viewer, model_projection, 0, 255, 0, 3.0f, ss.str());
  //  //  }
  //
  //  //  cv::namedWindow("out", cv::WINDOW_NORMAL);
  //  //  cv::imshow("out", out);
  //  //  cv::namedWindow("out_scene", cv::WINDOW_NORMAL);
  //  //  cv::imshow("out_scene", out_scene);
  //
  //  while (!viewer->wasStopped())
  //  {
  //    cv::waitKey(100);
  //    viewer->spinOnce();
  //  }



  return 1;
}
