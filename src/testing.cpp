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

/*
 *
 */
int
main (int argc, char** argv)
{

  cv::Mat model_rgb, model_rgb_full;
  pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr model_cloud_full(new pcl::PointCloud<PointType>());
  Eigen::Matrix4f model_pose;

  //LOAD MODEL
  visy::dataset::IrosDataset::loadModel(
          "asus_box", 1,
          model_cloud_full, model_cloud,
          model_rgb, model_rgb_full,
          model_pose);


  cv::imshow("ciao", model_rgb_full);
  cv::waitKey(0);

  visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor();

  std::vector<visy::extractors::KeyPoint3D> keypoints;
  extractor->extract(model_rgb_full, model_cloud_full, keypoints);

  cv::Mat out = model_rgb_full.clone();

  std::cout << "FIND:" << keypoints.size() << std::endl;


  visy::extractors::Bold3DExtractor::draw3DKeyPointsWithAreas(out, keypoints, cv::Scalar(255, 0, 0), 1.0f,5.0f,2.0f);

  pcl::visualization::PCLVisualizer * viewer;
  viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");

  
 viewer->addPointCloud(model_cloud_full, "model");
  visy::extractors::KeyPoint3D::draw3DKeyPoints3D(*viewer, keypoints, cv::Scalar(0, 255, 0), "ciao");

cv::namedWindow("out",cv::WINDOW_NORMAL);  
  cv::imshow("out", out);

  while (!viewer->wasStopped())
  {
    cv::waitKey(100);
    viewer->spinOnce();
  }



  std::cout << "EDGE DONE" << std::endl;



  return 1;
}
