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

using namespace std;
using namespace BoldLib;

cv::Mat scene_rgb, scene_rgb_full;
pcl::PointCloud<PointType>::Ptr scene_cloud (new pcl::PointCloud<PointType>());
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

  parameters = new visy::Parameters(argc, argv);
  parameters->putString("bunch_method");
  parameters->putFloat("radius");
  parameters->putFloat("size");


  //  cv::Mat model_rgb, model_rgb_full;
  //  pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
  //  pcl::PointCloud<PointType>::Ptr model_cloud_full(new pcl::PointCloud<PointType>());
  //  Eigen::Matrix4f model_pose;
  //
  //  //LOAD MODEL
  //  visy::dataset::IrosDataset::loadModel(
  //          "asus_box", 1,
  //          model_cloud_full, model_cloud,
  //          model_rgb, model_rgb_full,
  //          model_pose);





  visy::dataset::IrosDataset::loadScene(14, 6, scene_cloud, scene_rgb);

  //  visy::dataset::IrosDataset::loadModel(
  //          "asus_box", 2,
  //          scene_cloud, scene_cloud_full,
  //          scene_rgb_full, scene_rgb,
  //          scene_pose);


  std::vector<float> sizes;
  sizes.push_back(search_size);
  //  sizes.push_back(10);
  //  sizes.push_back(15);
  //  sizes.push_back(20);
  //  sizes.push_back(25);
  //  sizes.push_back(30);

  std::vector<float> radiuses;
  //  radiuses.push_back(0.001f);
  radiuses.push_back(search_radius);
  //  radiuses.push_back(0.1f);

  visy::detectors::Detector * detector;


  if (parameters->getString("bunch_method")=="knn")
  {
    detector = new visy::detectors::Bold3DMDetector(sizes, 8, true);
  }
  else if (parameters->getString("bunch_method")=="radius")
  {
    detector = new visy::detectors::Bold3DRDetector(radiuses, 8, true);
  }

  std::cout << "USING DETECTOR: " << detector->buildName() << std::endl;

  detector->detect(scene_rgb, scene_cloud, scene_keypoints, scene_descriptor);


  viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");
  viewer->addPointCloud(scene_cloud, "scene");
  cv::namedWindow("out", cv::WINDOW_NORMAL);
  cv::setMouseCallback("out", CallBackFunc, NULL);


  redraw();
  while (!viewer->wasStopped())
  {
    cv::waitKey(100);
    viewer->spinOnce();
  }

  return 1;
}
