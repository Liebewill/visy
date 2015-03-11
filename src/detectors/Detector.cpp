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
    }

    Detector::~Detector ()
    {
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