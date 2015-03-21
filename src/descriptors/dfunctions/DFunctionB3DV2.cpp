/* 
 * File:   DFunctionB3DV2.cpp
 * Author: daniele
 * 
 * Created on 15 marzo 2015, 13.54
 */

#include "dfunctions/DFunctionB3DV2.h"
#include "signatures/Signature.h"

namespace visy
{
  namespace descriptors
  {

    DFunctionB3DV2::DFunctionB3DV2 (int n_bins) : DFunction ()
    {
      this->size = 3;
      this->signature = new Signature(3, 180.0f, n_bins, false);
    }

    DFunctionB3DV2::~DFunctionB3DV2 ()
    {
    }

    void
    DFunctionB3DV2::f (visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results)
    {
      float* v = new float[this->size];

      cv::Point3f v1 = kp1.direction_x;
      cv::Point3f v2 = kp2.direction_x;

      cv::Point3f vm = kp2.pt3D - kp1.pt3D;


      cv::Point3f na = kp1.direction_z;
      cv::Point3f nb = kp2.direction_z;

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
    DFunctionB3DV2::buildStringImpl ()
    {
      return "B3DV2";
    }

  }
}