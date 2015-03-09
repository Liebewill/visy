/* 
 * File:   Bold3DExtractor.cpp
 * Author: daniele
 * 
 * Created on 9 marzo 2015, 18.14
 */

#include "Bold3DExtractor.h"
#include "tools.h"

namespace visy
{
  namespace extractors
  {

    Bold3DExtractor::Bold3DExtractor (float zone_radius, float zone_slice) : Extractor ()
    {
      this->zone_radius = zone_radius;
      this->zone_slice = zone_slice;
    }

    Bold3DExtractor::Bold3DExtractor (const Bold3DExtractor& orig)
    {
      this->zone_radius = orig.zone_radius;
      this->zone_slice = orig.zone_slice;
    }

    Bold3DExtractor::~Bold3DExtractor ()
    {
    }

    void
    Bold3DExtractor::bindArea (KeyPoint3D& kp, cv::Size2i source_size, std::vector<int>& area, float radius, float slice)
    {
      int max = source_size.width * source_size.height - 1;
      area.clear();
      cv::Point2i tl, br;
      int index = -1;
      int startx = kp.pt.x - kp.size;
      int starty = kp.pt.y - kp.size;
      startx = startx < 0 ? 0 : startx;
      starty = starty < 0 ? 0 : starty;
      for (int y = starty; y < starty + kp.size * 2; y++)
      {
        for (int x = startx; x < startx + kp.size * 2; x++)
        {
          cv::Point2f tp(x, y);
          double dis = cv::norm(kp.pt - tp);
          if (dis <= radius)
          {
            double slice_dis = visy::tools::point2LineDistance(kp.pt1, kp.pt2, tp);
            if (slice_dis > slice)
            {
              index = tp.x + tp.y * source_size.width;
              if (index > max)index = max;
              if (index < 0) index = 0;
              area.push_back(index);
            }
          }
        }
      }
    }

    void
    Bold3DExtractor::bindAreas (KeyPoint3D& kp, cv::Size2i source_size, std::vector<int>& area_left, std::vector<int>& area_right, float radius, float slice)
    {
      std::vector<int> area;
      bindArea(kp,source_size, area, radius, slice);

      area_left.clear();
      area_right.clear();


      for (int i = 0; i < area.size(); i++)
      {
        int index = area[i];
        cv::Point2i tp(index % source_size.width, index / source_size.width);
        cv::Point2i tp_1 = kp.pt1 - tp;
        cv::Point2i tp_2 = kp.pt2 - tp;
        int dot = tp_1.cross(tp_2);
        if (dot >= 0)
        {
          area_right.push_back(index);
        }
        else
        {
          area_left.push_back(index);
        }
      }
    }
    
    void
    Bold3DExtractor::extract (cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<KeyPoint3D>& keypoints, cv::Mat* mask)
    {
      std::vector<cv::Vec4f> lines;
      visy::tools::edgeDetection(source, lines, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_LSD);

      keypoints.clear();

      float x, y;
      std::vector<int> area_left,area_right;
      for (unsigned int i = 0; i < lines.size(); i++)
      {

        x = (lines[i][2] + lines[i][0]) / 2.0f;
        y = (lines[i][3] + lines[i][1]) / 2.0f;

        KeyPoint3D kp3D(lines[i], cloud);
        keypoints.push_back(kp3D);
        
        
        area_left.clear();
        area_right.clear();
        pcl::PointCloud<PointType>::Ptr normals_left(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr normals_right(new pcl::PointCloud<PointType>());
        
        
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
        bindAreas(kp,cv::Size2i(out.cols, out.rows), area_left, area_right, radius, slice);
        
        for (int area_index = 0; area_index < area_left.size(); area_index++)
        { 
          cv::circle(out,cv::Point(area_left[area_index] % out.cols, area_left[area_index] / out.cols),1.0f,cv::Scalar(0,0,255));
        }
        
        for (int area_index = 0; area_index < area_right.size(); area_index++)
        { 
          cv::circle(out,cv::Point(area_right[area_index] % out.cols, area_right[area_index] / out.cols),1.0f,cv::Scalar(0,255,0));
        }
      }
      
    }

  }
}