/* 
 * File:   DFunctionB3DV1.cpp
 * Author: daniele
 * 
 * Created on 15 marzo 2015, 13.54
 */

#include "DFunctionB3DV1.h"

namespace visy
{
  namespace descriptors
  {

    DFunctionB3DV1::DFunctionB3DV1 (int n_bins) : DFunction ()
    {
      this->size = 3;
      this->signature = new Signature(3, 180.0f, n_bins, false);
    }

    DFunctionB3DV1::~DFunctionB3DV1 ()
    {
    }

    void
    DFunctionB3DV1::f (visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results)
    {
      float* v = new float[this->size];

      cv::Point3f v1 = kp1.direction_x;
      cv::Point3f v2 = kp2.direction_x;

      cv::Point3f vm = kp2.pt3D - kp1.pt3D;


      cv::Point3f na = v1.cross(vm);
      cv::Point3f nb = v2.cross(vm);

      float alpha = v1.dot(vm);
      float beta = v2.dot(vm);
      float gamma = na.dot(nb);

      float alpha_c = acos(alpha / (cv::norm(v1) * cv::norm(vm)));
      float beta_c = acos(beta / (cv::norm(v2) * cv::norm(vm)));
      float gamma_c = acos(gamma / (cv::norm(na) * cv::norm(nb)));

      v[0] = alpha_c * (180 / M_PI);
      v[1] = beta_c * (180 / M_PI);
      v[2] = gamma_c * (180 / M_PI);

      *results = v;
    }

    std::string
    DFunctionB3DV1::buildStringImpl ()
    {
      return "B3DV1";
    }

  }
}