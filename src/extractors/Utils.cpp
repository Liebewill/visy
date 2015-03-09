/* 
 * File:   Utils.cpp
 * Author: daniele
 * 
 * Created on 9 marzo 2015, 21.43
 */

#include "Utils.h"

namespace visy
{
  namespace extractors
  {
    namespace utils
    {

      void
      extractSliceAreaFromKeypoint3D (KeyPoint3D& kp, cv::Size2i source_size, std::vector<int>& area, float radius, float slice)
      {
        int max = source_size.width * source_size.height - 1;
        area.clear();
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
      extractSliceAreaPairFromKeypoint3D (KeyPoint3D& kp, cv::Size2i source_size, std::vector<int>& area_left, std::vector<int>& area_right, float radius, float slice)
      {
        std::vector<int> area;
        extractSliceAreaFromKeypoint3D(kp, source_size, area, radius, slice);

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

    }
  }
}