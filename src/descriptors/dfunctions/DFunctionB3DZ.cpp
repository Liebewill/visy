/* 
 * File:   DFunctionB3DZ.cpp
 * Author: daniele
 * 
 * Created on April 7, 2015, 3:45 PM
 */

#include "DFunctionB3DZ.h"
namespace visy {
    namespace descriptors {

        DFunctionB3DZ::DFunctionB3DZ(int n_bins, bool multi) {
            this->size = 3;
            this->sparse = multi;
            this->signature = new Signature(3, 180.0f, n_bins, multi);
        }

        DFunctionB3DZ::~DFunctionB3DZ() {
        }

        void
        DFunctionB3DZ::f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results) {
            float* v = new float[this->size];

            cv::Point3f v1 = kp1.direction_x;
            cv::Point3f v2 = kp2.direction_x;
            cv::Point3f diag = kp2.pt3D_1 - kp1.pt3D_2;

            cv::Point3f vm = kp2.pt3D - kp1.pt3D;


            cv::Point3f na = kp1.direction_z;
            cv::Point3f nb = kp2.direction_z;

            float alpha = v1.dot(vm);
            float beta = v2.dot(vm);
            float gamma = v1.dot(diag);

            float alpha_c = acos(alpha / (cv::norm(v1) * cv::norm(vm)));
            float beta_c = acos(beta / (cv::norm(v2) * cv::norm(vm)));
            float gamma_c = acos(gamma / (cv::norm(v1) * cv::norm(diag)));

            v[0] = alpha_c * (180 / M_PI);
            v[1] = beta_c * (180 / M_PI);
            v[2] = gamma_c * (180 / M_PI);

            *results = v;
        }

        std::string
        DFunctionB3DZ::buildStringImpl() {
            std::stringstream ss;
            ss << "B3DZ";
            if(this->sparse){
                ss<<"_MULTI";
            }
            return ss.str();
        }
    }
}