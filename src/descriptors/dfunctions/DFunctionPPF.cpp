/* 
 * File:   DFunctionPPF.cpp
 * Author: daniele
 * 
 * Created on April 20, 2015, 11:53 AM
 */

#include "DFunctionPPF.h"

namespace visy {
    namespace descriptors {

        DFunctionPPF::DFunctionPPF(int n_bins) {
            this->size = 3;
            this->signature = new Signature(3, 180.0f, n_bins, false);
        }

        DFunctionPPF::~DFunctionPPF() {
        }

        void
        DFunctionPPF::f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results) {
            float* val = new float[this->size];

            cv::Point3f n1 = kp1.direction_z;
            cv::Point3f n2 = kp2.direction_z;
            cv::Point3f d = kp2.pt3D - kp1.pt3D;
           


            float alpha = n1.dot(d);
            float beta = n2.dot(d);
            float gamma = n1.dot(n2);

            float alpha_c = acos(alpha / (cv::norm(d)));
            float beta_c = acos(beta / (cv::norm(d)));
            float gamma_c = acos(gamma);

            val[0] = alpha_c * (180 / M_PI);
            val[1] = beta_c * (180 / M_PI);
            val[2] = gamma_c * (180 / M_PI);
            *results = val;
        }

        std::string
        DFunctionPPF::buildStringImpl() {
            return "PPF";
        }
    }
}
