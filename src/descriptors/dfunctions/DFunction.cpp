/* 
 * File:   DFunction.cpp
 * Author: daniele
 * 
 * Created on 15 marzo 2015, 13.49
 */

#include "DFunction.h"

namespace visy
{
  namespace descriptors
  {

    DFunction::DFunction ()
    {
      this->size = -1;
      this->signature = NULL;
    }

    DFunction::~DFunction ()
    {
      if (this->signature != NULL)
        delete this->signature;
    }

    int
    DFunction::getDataSize ()
    {
      if (this->signature != NULL)
      {
        return this->signature->getSize();
      }
      else
      {
        return -1;
      }
    }

    int
    DFunction::getSize ()
    {
      return this->size;
    }

    Signature*
    DFunction::getSignature ()
    {
      return this->signature;
    }

    std::string
    DFunction::buildString ()
    {
      std::stringstream ss;
      ss << "DF_" << this->buildStringImpl();
      return ss.str();
    }

  }
}