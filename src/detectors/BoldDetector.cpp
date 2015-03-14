/* 
 * File:   BoldDetector.cpp
 * Author: daniele
 * 
 * Created on 11 marzo 2015, 18.05
 */

#include "BoldDetector.h"

namespace visy
{
  namespace detectors
  {

    BoldDetector::BoldDetector (std::vector<float>& sizes, int n_bins, int scale_levels, float scale_space_factor) : Detector()
    {
      BoldLib::BOLD bold;
      
      std::vector<int> isizes;
      for(int i =0; i < sizes.size(); i++){
        isizes.push_back(sizes[i]);
      }
      bold.setK(isizes);
      bold.setNumAngularBins(n_bins);
      bold.setScaleSpaceLevels(scale_levels);
      bold.setScaleSpaceScaleFactor(scale_space_factor);
    }

    void
    BoldDetector::detect (cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask)
    {
      cv::Mat gray;
      if (source.type() != 0)
      {
        cv::cvtColor(source, gray, CV_BGR2GRAY);
      }
      else
      {
        gray = source;
      }

      BoldLib::Desc desc;
      BoldLib::Keypoint* temp_keypoints;
      int descriptorSize = bold.getNumAngularBins() * bold.getNumAngularBins();
      int nkps = bold.extract(gray, temp_keypoints, desc);

      descriptor = cv::Mat::zeros(nkps, descriptorSize, CV_32F);

      for (int i = 0; i < nkps; i++)
      {
        cv::Vec4i line;
        BoldLib::Keypoint boldKP = temp_keypoints[i];
        float angle = -boldKP.orientation;
        float x = boldKP.x;
        float y = boldKP.y;
        float med_x = boldKP.scale * 0.5f * cos((angle) * M_PI / 180.0f);
        float med_y = boldKP.scale * 0.5f * sin((angle) * M_PI / 180.0f);
        line.val[0] = x - med_x;
        line.val[1] = y - med_y;
        line.val[2] = x + med_x;
        line.val[3] = y + med_y;

        visy::extractors::KeyPoint3D kp3d(line, cloud);
        
        for (int k = 0; k < descriptorSize; k++)
        {
          descriptor.at<float>(i,k) = desc.getDoubleReadOnly()[i][k];
        }
        
        keypoints.push_back(kp3d);
      }

    }

    BoldDetector::~BoldDetector ()
    {
    }

    std::string
    BoldDetector::buildNameImpl ()
    {
      return "BOLD";
    }

  }
}