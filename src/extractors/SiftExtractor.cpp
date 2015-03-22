/* 
 * File:   SiftExtractor.cpp
 * Author: daniele
 * 
 * Created on 21 marzo 2015, 22.58
 */

#include <opencv2/nonfree/features2d.hpp>

#include "SiftExtractor.h"
#include "extrators_utils.h"
namespace visy
{
  namespace extractors
  {

    SiftExtractor::SiftExtractor ()
    {

    }

    SiftExtractor::~SiftExtractor ()
    {
    }

    void
    SiftExtractor::extract (cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<KeyPoint3D>& keypoints, cv::Mat* mask)
    {
      cv::SiftFeatureDetector detector;

      std::vector<cv::KeyPoint> cv_keypoints;
      std::vector<KeyPoint3D> keypoints_temp;

      detector.detect(source, cv_keypoints);

      float x, y;
      std::vector<int> area_left, area_right;
      for (int i = 0; i < cv_keypoints.size(); i++)
      {
        KeyPoint3D kp(cv_keypoints[i], cloud);


        //LEFT/RIGHT area calculation
        area_left.clear();
        area_right.clear();
        visy::extractors::utils::extractSliceAreaPairFromKeypoint3D(kp, cv::Size2i(source.cols, source.rows), area_left, area_right, 5.0f, 0.0f);

        if (area_left.size() < 3 || area_right.size() < 3)
        {
          kp.type = KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION;
        }
        else
        {

          //Compute AREA NORMALS
          Eigen::Vector3f normal_vector_left, normal_vector_right;
          visy::tools::planeNormalVector(cloud, area_left, normal_vector_left);
          visy::tools::planeNormalVector(cloud, area_right, normal_vector_right);

          kp.direction_z.x = normal_vector_left[0];
          kp.direction_z.y = normal_vector_left[1];
          kp.direction_z.z = normal_vector_left[2];


          //Checks Angle bitween normals
          float span = acos(normal_vector_left.dot(normal_vector_right));
          span = span * (180 / M_PI);
          if (span > 25.0f)
          {
            kp.type = KeyPoint3D::KEYPOINT3D_TYPE_EDGE_SURFACE;
          }
          else
          {
            kp.type = KeyPoint3D::KEYPOINT3D_TYPE_EDGE_TEXTURE;
          }

          //Computes Area Centroid
          Eigen::Vector4f centroid_left, centroid_right;
          pcl::PointCloud<PointType>::Ptr cloud_left = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType> ());
          pcl::PointCloud<PointType>::Ptr cloud_right = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType> ());
          pcl::copyPointCloud(*cloud, area_left, *cloud_left);
          pcl::copyPointCloud(*cloud, area_right, *cloud_right);
          pcl::compute3DCentroid(*cloud_left, centroid_left);
          pcl::compute3DCentroid(*cloud_right, centroid_right);

          Eigen::Vector4f centroid_distance = centroid_left - centroid_right;


          cv::Point3f direction_left, direction_right, direction;
          direction_right.x = centroid_right[0] - kp.pt3D.x;
          direction_right.y = centroid_right[1] - kp.pt3D.y;
          direction_right.z = centroid_right[2] - kp.pt3D.z;

          direction_left.x = centroid_left[0] - kp.pt3D.x;
          direction_left.y = centroid_left[1] - kp.pt3D.y;
          direction_left.z = centroid_left[2] - kp.pt3D.z;

          float step = 0.0f;
          if (direction_right.dot(direction_right) > direction_left.dot(direction_left))
          {
            direction = direction_left;
            step = direction_right.dot(direction_right);
          }
          else
          {
            direction = direction_right;
            step = direction_left.dot(direction_left);
          }
          step = step < 0 ? -step : step;
          if (step > 0.001f || isnan(step))
          {
            kp.type = KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION;
            kp.direction_y = direction;
          }
          else
          {
            kp.direction_y = direction_right;
          }
        }
        keypoints_temp.push_back(kp);
      }

      if (false)
      {
        for (int i = 0; i < keypoints_temp.size(); i++)
        {
          if (keypoints_temp[i].type == KeyPoint3D::KEYPOINT3D_TYPE_EDGE_SURFACE || keypoints_temp[i].type == KeyPoint3D::KEYPOINT3D_TYPE_EDGE_TEXTURE)
            keypoints.push_back(keypoints_temp[i]);
        }
      }
      else
      {
        keypoints.insert(keypoints.end(), keypoints_temp.begin(), keypoints_temp.end());
      }

      keypoints_temp.clear();
    }

    std::string
    SiftExtractor::buildNameImpl ()
    {
      return "SIFT";
    }


  }
}
