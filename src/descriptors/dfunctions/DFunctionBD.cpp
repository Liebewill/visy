/* 
 * File:   DFunctionBD.cpp
 * Author: daniele
 * 
 * Created on May 16, 2015, 12:58 AM
 */

#include "DFunctionBD.h"
namespace visy {
    namespace descriptors {

        DFunctionBD::DFunctionBD(int n_bins, bool multi) {
            this->size = 4;
            this->sparse = multi;
            this->signature = new Signature(4, 180.0f, n_bins, multi);
        }

        DFunctionBD::~DFunctionBD() {
        }

        std::string DFunctionBD::buildStringImpl() {
            return "BD";
        }

        void DFunctionBD::f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results) {
            float* v = new float[this->size];

            cv::Point3f v1 = kp1.direction_x;
            cv::Point3f v2 = kp2.direction_x;
            cv::Point3f diag = kp2.pt3D_1 - kp1.pt3D_2;

            cv::Point3f vm = kp2.pt3D - kp1.pt3D;
            cv::Point3f d2 = kp2.pt3D_1 - kp2.pt3D_2;
            
            float dvm = cv::norm(vm);
            float dd2 = cv::norm(d2);

            
            float alpha = v1.dot(vm);
            float beta = v2.dot(vm);
            float gamma = dvm;
            float delta = dd2;

            float alpha_c = acos(alpha / (cv::norm(v1) * cv::norm(vm)));
            float beta_c = acos(beta / (cv::norm(v2) * cv::norm(vm)));

            v[0] = alpha_c * (180 / M_PI);
            v[1] = beta_c * (180 / M_PI);
            v[2] = gamma * 500 <= 180.0f ? gamma * 500 : 180.0f;
            v[3] = delta * 500 <= 180.0f ? delta * 500 : 180.0f;
            
            *results = v;
        }

    }
}
