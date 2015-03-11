/* 
 * File:   Extractor.cpp
 * Author: daniele
 * 
 * Created on 9 marzo 2015, 18.11
 */

#include "Extractor.h"

namespace visy
{
  namespace extractors
  {

    Extractor::Extractor ()
    {
    }

    Extractor::~Extractor ()
    {
    }

    std::string
    Extractor::buildName ()
    {
      std::stringstream ss;
      ss << "EXTRACTOR_" << this->buildNameImpl();
      return ss.str();
    }

  }
}

