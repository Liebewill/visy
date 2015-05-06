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

    Bold3DExtractor::Bold3DExtractor (bool filter_occlusion, float zone_radius, float zone_slice, float area_normals_angular_th, float area_max_distance, int extraction_method , int parallels_rounds, float parallels_distance) : Extractor ()
    {
      this->filter_occlusion = filter_occlusion;
      this->area_radius = zone_radius;
      this->area_slice = zone_slice;
      this->area_normals_angular_th = area_normals_angular_th;
      this->area_max_distance = area_max_distance;
      this->extraction_method = extraction_method;
      this->parallels_rounds = parallels_rounds;
      this->parallels_distance = parallels_distance;
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
      visy::tools::edgeDetection(source, lines, this->extraction_method);

      std::vector<KeyPoint3D> keypoints_temp;

      float x, y;
      std::vector<int> area_left, area_right;
      for (unsigned int i = 0; i < lines.size(); i++)
      {

        x = (lines[i][2] + lines[i][0]) / 2.0f;
        y = (lines[i][3] + lines[i][1]) / 2.0f;

        KeyPoint3D kp(lines[i], cloud);


        checkKeyPoint3DType(source, cloud, kp);
        keypoints_temp.push_back(kp);
      }

      if (filter_occlusion)
      {
        for (int i = 0; i < keypoints_temp.size(); i++)
        {
          if (keypoints_temp[i].type == KeyPoint3D::KEYPOINT3D_TYPE_EDGE_SURFACE || keypoints_temp[i].type == KeyPoint3D::KEYPOINT3D_TYPE_EDGE_TEXTURE)
          {
            keypoints.push_back(keypoints_temp[i]);
          }
          else
          {

            /** Calculates parallels */
            float angle = keypoints_temp[i].angle * M_PI / 180.0f;
            cv::Point2f orientation(cos(angle), sin(angle));
            cv::Point2f perp(-orientation.y, orientation.x);
            perp.x = perp.x / cv::norm(perp);
            perp.y = perp.y / cv::norm(perp);

            std::vector<visy::extractors::KeyPoint3D> parallels_kps;
            std::vector<visy::extractors::KeyPoint3D> parallels_kps_filtered;

            float pd = this->parallels_distance;
            int parallels_rounds = this->parallels_rounds;
            
            for (int pe = -parallels_rounds; pe <= parallels_rounds; pe++)
            {
              if (pe == 0)continue;

              cv::Point2f distf = perp * pd * (pe + 1);
              visy::extractors::KeyPoint3D kp3d = keypoints_temp[i].cloneTranslated(cloud, distf);
              checkKeyPoint3DType(source, cloud, kp3d);
              parallels_kps.push_back(kp3d);
            }

            /* BEST PARALLEL KEYPOINT*/

            std::sort(parallels_kps.begin(), parallels_kps.end(), visy::extractors::KeyPoint3DZOrderer());

            /* filtering ext edges*/
            for (int sp = 0; sp < parallels_kps.size(); sp++)
            {
              if (parallels_kps[sp].type != visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION_EXT)
              {
                parallels_kps_filtered.push_back(parallels_kps[sp]);
              }
            }

            /** Find best */

            cv::Point3f last_distance_v;
            float last_distance = -1.0f;
            int sel_index = -1;
            for (int sp = 0; sp < parallels_kps_filtered.size(); sp++)
            {
              if (sp == 0)continue;
              last_distance_v = parallels_kps_filtered[sp].pt3D - parallels_kps_filtered[sp - 1].pt3D;
              last_distance = cv::norm(last_distance_v);
              if (last_distance > this->area_max_distance / 2.0f)
              {
                sel_index = sp - 1;
                break;
              }
            }
            if (sel_index < 0)
            {
              sel_index = parallels_kps_filtered.size() - 1;
            }

            if (sel_index >= 0)
              if (parallels_kps_filtered[sel_index].type != visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION_EXT)
              {
                keypoints.push_back(parallels_kps_filtered[sel_index]);
              }

          }
        }
      }
      else
      {
        for (int i = 0; i < keypoints_temp.size(); i++)
        {
          //          if (keypoints_temp[i].type == KeyPoint3D::KEYPOINT3D_TYPE_EDGE_SURFACE || keypoints_temp[i].type == KeyPoint3D::KEYPOINT3D_TYPE_EDGE_TEXTURE || keypoints_temp[i].type == KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION)
          keypoints.push_back(keypoints_temp[i]);
        }
      }

      keypoints_temp.clear();

    }

    void
    Bold3DExtractor::checkKeyPoint3DType (cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, visy::extractors::KeyPoint3D& kp)
    {
      if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_INVALID)return;

      float x, y;
      std::vector<int> area_left, area_right;
      //LEFT/RIGHT area calculation
      area_left.clear();
      area_right.clear();
      visy::extractors::utils::extractSliceAreaPairFromKeypoint3D(kp, cv::Size2i(source.cols, source.rows), area_left, area_right, this->area_radius, this->area_slice);

      if (area_left.size() < 3 || area_right.size() < 3)
      {
        kp.type = KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION_EXT;
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

        float step = 0.0f;
        if (cv::norm(direction_right) > cv::norm(direction_left))
        {
          direction = direction_left;
          step = cv::norm(direction_left);
        }
        else
        {
          direction = direction_right;
          step = cv::norm(direction_right);
        }
        step = step < 0 ? -step : step;
        if (cv::norm(direction_left)> this->area_max_distance || cv::norm(direction_right)>this->area_max_distance)
        {
          kp.type = KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION;
          kp.direction_y = direction;
        }

        if (cv::norm(direction_left)> this->area_max_distance && cv::norm(direction_right)>this->area_max_distance)
        {
          kp.type = KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION_EXT;
        }

        if (cv::norm(direction_left) <= this->area_max_distance && cv::norm(direction_right) <= this->area_max_distance)
        {
          kp.direction_y = direction_right;
        }



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

    std::string
    Bold3DExtractor::buildNameImpl ()
    {
      std::stringstream ss;
      ss << "BOLD3D;";
      ss << "E" << this->extraction_method << ";";
      ss << this->filter_occlusion << ";";
      ss << this->area_radius << ";";
      ss << this->area_slice << ";";
      ss << this->area_normals_angular_th << ";";
      ss << this->area_max_distance;
      return ss.str();
    }

  }
}