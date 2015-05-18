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
#include "detectors/detectors_utils.h"

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

using namespace std;
using namespace BoldLib;

cv::Mat scene_rgb, scene_rgb_full;
pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr scene_cloud_full (new pcl::PointCloud<PointType>());
Eigen::Matrix4f scene_pose;
pcl::visualization::PCLVisualizer * viewer;
cv::Mat out;
std::vector<visy::extractors::KeyPoint3D> model_keypoints;
std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
std::vector<visy::extractors::KeyPoint3D> target_keypoints;
std::vector<visy::extractors::KeyPoint3D> bunch_keypoints;
cv::Mat model_descriptor;
cv::Mat scene_descriptor;
int search_size = 5;
float search_radius = 0.15f;
visy::Parameters * parameters;

void redraw ();

int
searchNearestKeypoint (int x, int y, std::vector<visy::extractors::KeyPoint3D>& keypoints)
{

  float min_distance = 100000.0f;
  int min_index = -1;

  cv::Point2f spoint(x, y);
  for (int i = 0; i < keypoints.size(); i++)
  {

    cv::Point2f dd = spoint - keypoints[i].pt;
    float d = dd.dot(dd);
    if (d < min_distance)
    {
      min_distance = d;
      min_index = i;
    }

  }

  return min_index;

}

int
bunchSearch (int search_index, std::vector<visy::extractors::KeyPoint3D>& keypoints, float size, bool radius)
{
  pcl::KdTreeFLANN<PointType> kdtree;
  pcl::PointCloud<PointType>::Ptr keypoints_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType> ());
  visy::extractors::utils::buildPrimiteCloudFromKeypoints(keypoints_cloud, keypoints);
  kdtree.setInputCloud(keypoints_cloud);

  std::vector<int> found_indices;
  std::vector<float> indices_distances;
  PointType searchPoint;

  searchPoint.x = keypoints[search_index].pt3D.x;
  searchPoint.y = keypoints[search_index].pt3D.y;
  searchPoint.z = keypoints[search_index].pt3D.z;
  std::cout << "Searcging around " << keypoints[search_index].pt3D << std::endl;
  if (radius)
  {
    kdtree.radiusSearch(searchPoint, size, found_indices, indices_distances);
  }
  else
  {
    kdtree.nearestKSearch(searchPoint, size + 1, found_indices, indices_distances);


  }
  std::cout << "Found: " << found_indices.size() << std::endl;
  bunch_keypoints.clear();
  for (int i = 0; i < found_indices.size(); i++)
  {
    std::cout << "-- " << keypoints[found_indices[i]].pt3D << std::endl;
    bunch_keypoints.push_back(keypoints[found_indices[i]]);
  }


}

void
redraw ()
{

  viewer->removeAllShapes();
  visy::extractors::KeyPoint3D::draw3DKeyPoints3D(*viewer, bunch_keypoints, cv::Scalar(0, 255, 0), "ciao");
  out = scene_rgb.clone();
  visy::extractors::KeyPoint3D::draw3DKeyPoints(out, scene_keypoints, cv::Scalar(0, 255, 255), 1.0f);
  visy::extractors::KeyPoint3D::draw3DKeyPoints(out, target_keypoints, cv::Scalar(255, 0, 255), 1.0f, true);

  cv::imshow("out", out);
}

void
CallBackFunc (int event, int x, int y, int flags, void* userdata)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    int index = searchNearestKeypoint(x, y, scene_keypoints);
    target_keypoints.clear();
    target_keypoints.push_back(scene_keypoints[index]);
    if(parameters->getString("bunch_method")=="radius"){
      bunchSearch(index, scene_keypoints, parameters->getFloat("radius"), true);
    }else  if(parameters->getString("bunch_method")=="knn"){
      bunchSearch(index, scene_keypoints,  parameters->getFloat("size"), false);
    }
    
    redraw();
  }
  else if (event == cv::EVENT_RBUTTONDOWN)
  {
    target_keypoints.clear();
    bunch_keypoints.clear();
    redraw();
  }


}

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
    visy::detectors::Detector * detector_model;

    detector = visy::detectors::utils::buildDetectorFromString(parameters->getString("detector"), parameters, false);
    detector_model = visy::detectors::utils::buildDetectorFromString(parameters->getString("detector"), parameters, true);

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

    dataset.fetchFullModel(model.name, model.n_views, model_keypoints, model_descriptor, model_cloud, model_pose, detector_model);


    std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
    cv::Mat scene_descriptor;
    cv::Mat scene_rgb, scene_rgb_full;
    pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());

    visy::dataset::IrosDataset::loadScene(parameters->getInt("set"), parameters->getInt("scene"), scene_cloud, scene_rgb);
    
    viewer->addPointCloud(scene_cloud,"scene");
    

  while (!viewer->wasStopped())
  {
    cv::waitKey(100);
    viewer->spinOnce();
  }

  return 1;
}
