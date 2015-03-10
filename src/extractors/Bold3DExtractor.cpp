/* 
 * File:   Bold3DExtractor.cpp
 * Author: daniele
 * 
 * Created on 9 marzo 2015, 18.14
 */

#include "Bold3DExtractor.h"
#include "tools.h"
#include "extrators_utils.h"

namespace visy
{
  namespace extractors
  {

    Bold3DExtractor::Bold3DExtractor (float zone_radius, float zone_slice) : Extractor ()
    {
      this->area_radius = zone_radius;
      this->area_slice = zone_slice;
    }

    Bold3DExtractor::Bold3DExtractor (const Bold3DExtractor& orig)
    {
      this->area_radius = orig.area_radius;
      this->area_slice = orig.area_slice;
    }

    Bold3DExtractor::~Bold3DExtractor ()
    {
    }

    void
    Bold3DExtractor::extract (cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<KeyPoint3D>& keypoints, cv::Mat* mask)
    {
      std::vector<cv::Vec4f> lines;
      visy::tools::edgeDetection(source, lines, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_LSD);

      keypoints.clear();

      float x, y;
      std::vector<int> area_left, area_right;
      for (unsigned int i = 0; i < lines.size(); i++)
      {

        x = (lines[i][2] + lines[i][0]) / 2.0f;
        y = (lines[i][3] + lines[i][1]) / 2.0f;

        KeyPoint3D kp(lines[i], cloud);


        //LEFT/RIGHT area calculation
        area_left.clear();
        area_right.clear();
        visy::extractors::utils::extractSliceAreaPairFromKeypoint3D(kp, cv::Size2i(source.cols, source.rows), area_left, area_right, this->area_radius, this->area_slice);

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
          if (span > this->area_normals_angular_th)
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

          if (direction_right.dot(direction_right) < direction_left.dot(direction_left))
          {
            direction = direction_right;
          }
          else
          {
            direction = direction_left;
          } 
          
          float step = direction.dot(direction);
          step = step <0 ? -step:step;

          if(step> this->area_max_distance || isnan(step)){
            kp.type = KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION;
          }
          
          kp.direction_y = direction;
          

        }
        keypoints.push_back(kp);
      }



    }

    void
    Bold3DExtractor::draw3DKeyPointsWithAreas (cv::Mat& out, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Scalar color, float tick, float radius, float slice)
    {
      KeyPoint3D::draw3DKeyPoints(out, keypoints, color, tick);
      std::vector<int> area_left, area_right;
      for (int i = 0; i < keypoints.size(); i++)
      {
        area_left.clear();
        area_right.clear();
        visy::extractors::KeyPoint3D kp = keypoints[i];
        visy::extractors::utils::extractSliceAreaPairFromKeypoint3D(kp, cv::Size2i(out.cols, out.rows), area_left, area_right, radius, slice);

        for (int area_index = 0; area_index < area_left.size(); area_index++)
        {
          cv::circle(out, cv::Point(area_left[area_index] % out.cols, area_left[area_index] / out.cols), 1.0f, cv::Scalar(0, 0, 255));
        }

        for (int area_index = 0; area_index < area_right.size(); area_index++)
        {
          cv::circle(out, cv::Point(area_right[area_index] % out.cols, area_right[area_index] / out.cols), 1.0f, cv::Scalar(0, 255, 0));
        }

      }

    }

  }
}