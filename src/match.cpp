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


/*
LineSegmentDetection( int * n_out,
double * img, int X, int Y,
double scale, double sigma_scale, double quant,
double ang_th, double log_eps, double density_th,
int n_bins,
int ** reg_img, int * reg_x, int * reg_y );
 */

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
  parameters->putInt("set");
  parameters->putInt("scene");
  parameters->putInt("nbin");
  parameters->putInt("occlusion");

  int use_occlusion = false;

  visy::dataset::IrosDataset::init();
  visy::dataset::Model model = visy::dataset::IrosDataset::findModelByName(parameters->getString("model"));

  visy::detectors::Detector * detector;
  
  detector = visy::detectors::utils::buildDetectorFromString(parameters->getString("detector"),parameters);
  
  //  std::vector<float> sizes = visy::Parameters::parseFloatArray(parameters->getString("sizes"));
  //
  //
  //
  //  visy::detectors::Detector * detector;
  //
  //  if (parameters->getString("detector") == "BOLD3DM")
  //  {
  //    detector = new visy::detectors::Bold3DMDetector(sizes, parameters->getInt("nbin"), !use_occlusion);
  //  }
  //  else if (parameters->getString("detector") == "BOLD3DM2")
  //  {
  //    detector = new visy::detectors::Bold3DM2Detector(sizes, parameters->getInt("nbin"), !use_occlusion);
  //  }
  //  else if (parameters->getString("detector") == "BOLD3DM2MULTI")
  //  {
  //    detector = new visy::detectors::Bold3DM2MultiDetector(sizes, parameters->getInt("nbin"), !use_occlusion);
  //  }
  //  else if (parameters->getString("detector") == "BOLD3DR2")
  //  {
  //    detector = new visy::detectors::Bold3DR2Detector(sizes, parameters->getInt("nbin"), !use_occlusion);
  //  }
  //  else if (parameters->getString("detector") == "BOLD3DR")
  //  {
  //    detector = new visy::detectors::Bold3DRDetector(sizes, parameters->getInt("nbin"), !use_occlusion);
  //  }
  //  else if (parameters->getString("detector") == "BOLD")
  //  {
  //    detector = new visy::detectors::BoldDetector(sizes);
  //  }

  std::cout << "Detector: " << detector->buildName() << std::endl;

  pcl::visualization::PCLVisualizer * viewer;
  viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");

  visy::dataset::IrosDataset dataset;



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


  std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
  cv::Mat scene_descriptor;
  cv::Mat scene_rgb, scene_rgb_full;
  pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());

  visy::dataset::IrosDataset::loadScene(parameters->getInt("set"), parameters->getInt("scene"), scene_cloud, scene_rgb);

  detector->detect(scene_rgb, scene_cloud, scene_keypoints, scene_descriptor);




  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;

  visy::extractors::utils::modelSceneMatch(
          model_keypoints,
          scene_keypoints,
          model_descriptor,
          scene_descriptor,
          rototranslations,
          parameters->getFloat("gc_size"),
          parameters->getFloat("gc_th")
          );





  std::cout << "FOUND:" << rototranslations.size() << std::endl;


  for (int i = 0; i < rototranslations.size(); i++)
  {
    pcl::PointCloud<PointType>::Ptr model_projection(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*model_cloud, *model_projection, rototranslations[i]);
    std::stringstream ss;
    ss << "Instance_" << i << "_cloud";
    visy::tools::displayCloud(*viewer, model_projection, 0, 255, 0, 3.0f, ss.str());
  }



  viewer->addPointCloud(scene_cloud, "scene");


  //  visy::extractors::KeyPoint3D::draw3DKeyPoints3D(*viewer, model_keypoints, cv::Scalar(0, 255, 0), "ciao", true);

  //  for (int i = 0; i < rototranslations.size(); i++)
  //  {
  //    pcl::PointCloud<PointType>::Ptr model_projection(new pcl::PointCloud<PointType>());
  //    pcl::transformPointCloud(*model_cloud, *model_projection, rototranslations[i]);
  //    std::stringstream ss;
  //    ss << "Instance_" << i << "_cloud";
  //    visy::tools::displayCloud(*viewer, model_projection, 0, 255, 0, 3.0f, ss.str());
  //  }

  //  cv::namedWindow("out", cv::WINDOW_NORMAL);
  //  cv::imshow("out", out);
  //  cv::namedWindow("out_scene", cv::WINDOW_NORMAL);
  //  cv::imshow("out_scene", out_scene);

  while (!viewer->wasStopped())
  {
    cv::waitKey(100);
    viewer->spinOnce();
  }



  return 1;
}
