/* 
 * File:   Detector.cpp
 * Author: daniele
 * 
 * Created on 11 marzo 2015, 16.44
 */

#include <iosfwd>

#include "Detector.h"


namespace visy
{
  namespace detectors
  {

    Detector::Detector ()
    {
      this->extractor = NULL;
      this->descriptor = NULL;
      this->multiplication_factor = 1;
      this->detector_embedded = true;
    }

    Detector::~Detector ()
    {
      if (this->extractor != NULL)
      {
        delete this->extractor;
      }

      if (this->descriptor != NULL)
      {
        delete this->descriptor;
      }
    }
    int
    Detector::getMultiplicationFactor ()
    {
      return this->multiplication_factor;
    }

    bool
    Detector::isDetectorEmbedded ()
    {
      return this->detector_embedded;
    }

    void
    Detector::refineKeyPoints3D (std::vector<visy::extractors::KeyPoint3D>& keypoints_in, cv::Mat& descriptor_in, std::vector<visy::extractors::KeyPoint3D>& keypoints_out, cv::Mat& descriptor_out)
    {
      keypoints_out.insert(keypoints_out.end(), keypoints_in.begin(), keypoints_in.end());
      descriptor_out = descriptor_in;
    }

    void
    Detector::refineKeyPoints3D (std::vector<visy::extractors::KeyPoint3D>& keypoints_in, std::vector<visy::extractors::KeyPoint3D>& keypoints_out)
    {
      keypoints_out.insert(keypoints_out.end(), keypoints_in.begin(), keypoints_in.end());
    }

    std::string
    Detector::buildName ()
    {
      std::stringstream ss;
      ss << "DETECTOR_" << this->buildNameImpl() << "!";
      if (this->extractor != NULL)
      {
        ss << this->extractor->buildName();
      }
      ss << "!";

      if (this->descriptor != NULL)
      {
        ss << this->descriptor->buildName();
      }
      return ss.str();
    }



  }
}