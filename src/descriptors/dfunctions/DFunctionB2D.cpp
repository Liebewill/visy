/* 
 * File:   DFunctionB2D.cpp
 * Author: daniele
 * 
 * Created on 25 marzo 2015, 11.43
 */

#include "DFunctionB2D.h"

namespace visy
{
  namespace descriptors
  {

    DFunctionB2D::DFunctionB2D (int n_bins) : DFunction ()
    {
      this->size = 2;
      this->signature = new Signature(2, 180.0f, n_bins, false);
    }

    DFunctionB2D::~DFunctionB2D ()
    {
    }

    void
    DFunctionB2D::f (visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results)
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

      *results = v;
    }

    std::string
    DFunctionB2D::buildStringImpl ()
    {
      return "B2D";
    }
  }
}

