/* 
 * File:   Descriptor.cpp
 * Author: daniele
 * 
 * Created on 10 marzo 2015, 18.40
 */

#include "Descriptor.h"

namespace visy
{
  namespace descriptors
  {

    Descriptor::Descriptor (DFunction* dfunction)
    {
      this->dfunction = dfunction;
    }
    

    Descriptor::~Descriptor ()
    {
      if(this->dfunction!=NULL){
        delete this->dfunction;
      }
    }

    std::string
    Descriptor::buildName ()
    {
      std::stringstream ss;
      ss << "DESCRIPTOR_" << this->buildNameImpl();
      return ss.str();
    }


  }
}
