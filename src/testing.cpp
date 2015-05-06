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
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <sstream>
#include <string>
#include <fstream>

#include <tools.h>
#include <opencv2/core/mat.hpp>

#include "Extractor.h"
#include "Bold3DExtractor.h"
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
#include "commons/commons.h"
#include "WillowDataset.h"
#include "PipeLine.h"

using namespace std;
using namespace BoldLib;

visy::Parameters* parameters;

visy::extractors::Bold3DExtractor* ext;

/** SCENE */
std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
std::vector<visy::extractors::KeyPoint3D> scene_keypoints_seletected;
std::vector<visy::extractors::KeyPoint3D> scene_keypoints_parallels_all;
std::vector<visy::extractors::KeyPoint3D> scene_keypoints_parallels;
int scene_keypoints_parallels_best = -1;
std::vector<cv::Point2f> scene_keypoints_seletected_parallels;
int scene_keypoint_selected_index = -1;
cv::Mat scene_descriptor;
cv::Mat scene_rgb, scene_rgb_full;
pcl::PointCloud<PointType>::Ptr scene_cloud (new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr scene_cloud_filtered (new pcl::PointCloud<PointType>());

/**VIEWER*/
pcl::visualization::PCLVisualizer * viewer;

cv::Mat out, out_perp;

struct KeyPoint3DZOrderer
{

  inline bool operator() (const visy::extractors::KeyPoint3D& kp1,const visy::extractors::KeyPoint3D& kp2)
  {
    return kp1.pt3D.z < kp2.pt3D.z;
  }
};

void
draw3DKeyPointsColor (pcl::visualization::PCLVisualizer& viewer, cv::Mat& out, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Scalar color, std::string name, bool simple, bool parallels = false)
{

  pcl::PointCloud<PointType>::Ptr keypoint_cloud(new pcl::PointCloud<PointType>());
  pcl::PointCloud<NormalType>::Ptr keypoint_normals(new pcl::PointCloud<NormalType>());

  std::stringstream ss;
  for (int i = 0; i < keypoints.size(); i++)
  {
    visy::extractors::KeyPoint3D kp = keypoints[i];
    if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_TEXTURE)
    {
      color = cv::Scalar(0, 0, 255);
    }
    else if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_SURFACE)
    {
      color = cv::Scalar(0, 255, 0);
    }
    else if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION)
    {
      color = cv::Scalar(255, 0, 0);
    }
    else if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION_EXT)
    {
      color = cv::Scalar(255, 255, 255);
    }
    
    if(parallels && scene_keypoints_parallels_best == i){
      color = cv::Scalar(255,255,0);
    }


    PointType p;
    p.x = kp.pt3D.x;
    p.y = kp.pt3D.y;
    p.z = kp.pt3D.z;

    keypoint_cloud->points.push_back(p);

    Eigen::Vector3f vstart, vend;
    vstart << kp.pt3D.x - kp.direction_x.x / 2.0f, kp.pt3D.y - kp.direction_x.y / 2.0f, kp.pt3D.z - kp.direction_x.z / 2.0f;
    vend << kp.pt3D.x + kp.direction_x.x / 2.0f, kp.pt3D.y + kp.direction_x.y / 2.0f, kp.pt3D.z + kp.direction_x.z / 2.0f;


    ss.str("");
    ss << name << "_kp_x" << i;
    visy::tools::draw3DVector(viewer, vstart, vend, color[2] / 255.0f, color[1] / 255.0f, color[0] / 255.0f, ss.str());

    if (!simple)
    {
      float scale = 0.01f;
      vstart << kp.pt3D.x, kp.pt3D.y, kp.pt3D.z;
      vend << kp.pt3D.x + kp.direction_z.x *scale, kp.pt3D.y + kp.direction_z.y *scale, kp.pt3D.z + kp.direction_z.z *scale;

      ss.str("");
      ss << name << "_kp_z" << i;
      visy::tools::draw3DVector(viewer, vstart, vend, 0.0f, 0, 1.0f, ss.str());


      scale = 1.0f;
      vstart << kp.pt3D.x, kp.pt3D.y, kp.pt3D.z;
      vend << kp.pt3D.x + kp.direction_y.x *scale, kp.pt3D.y + kp.direction_y.y *scale, kp.pt3D.z + kp.direction_y.z *scale;

      ss.str("");
      ss << name << "_kp_y" << i;
      visy::tools::draw3DVector(viewer, vstart, vend, 0.0f, 1.0f, 0.0f, ss.str());
    }

  }

  visy::tools::displayCloud(viewer, keypoint_cloud, color[2], color[1], color[0], 5.0f, name);

  /**2D*/
  for (int i = 0; i < keypoints.size(); i++)
  {
    visy::extractors::KeyPoint3D kp = keypoints[i];

    cv::Point2f p1 = kp.pt - cv::Point2f(cos(kp.angle * M_PI / 180.0f) * kp.size / 2.0f, sin(kp.angle * M_PI / 180.0f) * kp.size / 2.0f);
    cv::Point2f p2 = kp.pt - cv::Point2f(-cos(kp.angle * M_PI / 180.0f) * kp.size / 2.0f, -sin(kp.angle * M_PI / 180.0f) * kp.size / 2.0f);

    if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_TEXTURE)
    {
      color = cv::Scalar(0, 0, 255);
    }
    else if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_SURFACE)
    {
      color = cv::Scalar(0, 255, 0);
    }
    else if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION)
    {
      color = cv::Scalar(255, 0, 0);
    }
    else if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION_EXT)
    {
      color = cv::Scalar(0, 0, 0);
    }

    float tick = 1.0f;

    if (!parallels)
    {
      if (scene_keypoint_selected_index >= 0 && scene_keypoint_selected_index == i)
      {

        tick = 2.0f;
        //color = cv::Scalar(255, 255, 0);

        float angle = scene_keypoints[i].angle * M_PI / 180.0f;
        std::cout << "Angle: " << angle << std::endl;
        cv::Point2f orientation(cos(angle), sin(angle));
        std::cout << "Or: " << orientation << std::endl;
        cv::Point2f perp(-orientation.y, orientation.x);
        perp.x = perp.x / cv::norm(perp);
        perp.y = perp.y / cv::norm(perp);

        float pd = 1.0f;

        scene_keypoints_parallels.clear();
        scene_keypoints_parallels_all.clear();
        for (int pe = -5; pe <= 5; pe++)
        {
          if (pe == 0)continue;

          cv::Point2f distf = perp * pd * (pe + 1);
          visy::extractors::KeyPoint3D kp3d = scene_keypoints[i].cloneTranslated(scene_cloud, distf);
          ext->checkKeyPoint3DType(scene_rgb, scene_cloud, kp3d);
          scene_keypoints_parallels_all.push_back(kp3d);
        }
        
        /* BEST KEYPOINT*/
        std::sort(scene_keypoints_parallels_all.begin(), scene_keypoints_parallels_all.end(), KeyPoint3DZOrderer());
        
        /* FILTERING EXT EDGES*/
        for (int sp = 0; sp < scene_keypoints_parallels_all.size(); sp++)
        {
          if(scene_keypoints_parallels_all[sp].type!=visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION_EXT){
            scene_keypoints_parallels.push_back(scene_keypoints_parallels_all[sp]);
          }
        }
        
        cv::Point3f last_distance_v;
        float last_distance = -1.0f;
        int sel_index = -1;
        for (int sp = 0; sp < scene_keypoints_parallels.size(); sp++)
        {
          if(sp==0  )continue;
          last_distance_v = scene_keypoints_parallels[sp].pt3D - scene_keypoints_parallels[sp - 1].pt3D;
          last_distance = cv::norm(last_distance_v);
          std::cout << "V"<<sp<< " "<<last_distance<<std::endl;
          if (last_distance > 0.03f)
          {
            sel_index = sp-1;
            break;
          }
        }
        if(sel_index<0){
          sel_index = scene_keypoints_parallels.size()-1;
        }
          scene_keypoints_parallels_best = sel_index;
        

      }
    }
    cv::line(out, p1, p2, color, tick);
    cv::circle(out, keypoints[i].pt, 3.0f, color, 1.0f);

  }

  if (!parallels && scene_keypoints_parallels.size() > 0)
  {

    viewer.removeAllShapes();
    draw3DKeyPointsColor(viewer, out_perp, scene_keypoints_parallels, cv::Scalar(255, 255, 0), "kp parallels", true, true);
  }
}

void
redraw ()
{
  out = scene_rgb.clone();
  out_perp = scene_rgb.clone();
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
  viewer->addPointCloud(scene_cloud, "scene");
  draw3DKeyPointsColor(*viewer, out, scene_keypoints, cv::Scalar(0, 255, 0), "scene_kps", true);
  cv::imshow("out", out);
  cv::imshow("out_perp", out_perp);
}

void
CallBackFunc (int event, int x, int y, int flags, void* userdata)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    std::cout << "Click " << x << std::endl;
    float min_dist = 1000.0f;
    int min_index = -1;

    for (int i = 0; i < scene_keypoints.size(); i++)
    {
      cv::Point2f d = scene_keypoints[i].pt - cv::Point2f(x, y);
      float dd = cv::norm(d);
      if (dd < min_dist)
      {
        min_dist = dd;
        min_index = i;
      }
    }

    if (min_index >= 0)
    {
      scene_keypoint_selected_index = min_index;
      scene_keypoints_seletected.clear();
      scene_keypoints_seletected.push_back(scene_keypoints[min_index]);
    }
    else
    {
      scene_keypoint_selected_index = -1;
    }
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
  parameters->putFloat("down");
  parameters->putString("dataset");
  parameters->putString("detector");
  parameters->putString("model");
  parameters->putString("sizes");
  parameters->putInt("set");
  parameters->putInt("scene");
  parameters->putInt("nbin");
  parameters->putInt("occlusion");
  parameters->putBool("x");


  viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");

  /** DATASET */
  std::cout << "Building Dataset: " << parameters->getString("dataset") << std::endl;
  visy::dataset::WillowDataset dataset(parameters->getString("dataset"));
  visy::dataset::Model model(parameters->getString("model"), 10);



  int set_number = parameters->getInt("set");
  int scene_number = parameters->getInt("scene");
  std::vector<visy::dataset::Annotation> annotations;
  dataset.loadScene(set_number, scene_number, scene_cloud, scene_rgb);
  dataset.loadAnnotiationsFromSceneFile(model.name, set_number, scene_number, annotations);


  ext = new visy::extractors::Bold3DExtractor(parameters->getBool("x"));
  
  std::cout << "Extracting"<<std::endl;
  ext->extract(scene_rgb, scene_cloud, scene_keypoints);

  std::cout << "Scene kps:" << scene_keypoints.size() << std::endl;




  out = scene_rgb.clone();



  cv::namedWindow("out", cv::WINDOW_NORMAL);
  cv::setMouseCallback("out", CallBackFunc, NULL);
  cv::namedWindow("out_perp", cv::WINDOW_NORMAL);
  cv::setMouseCallback("out_perp", CallBackFunc, NULL);

  redraw();

  while (!viewer->wasStopped())
  {
    cv::waitKey(100);
    viewer->spinOnce();
  }

  return 1;
}
