/* 
 * File:   DFunctionFPFH.cpp
 * Author: daniele
 * 
 * Created on April 2, 2015, 4:04 PM
 */

#include "DFunctionFPFH.h"
namespace visy {
    namespace descriptors {

        DFunctionFPFH::DFunctionFPFH(int n_bins, bool multi) {
            this->size = 3;
            this->signature = new Signature(3, 180.0f, n_bins, multi);
        }

        DFunctionFPFH::~DFunctionFPFH() {
        }

        void
        DFunctionFPFH::f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results) {
            float* val = new float[this->size];

            cv::Point3f n1 = kp1.direction_z;
            cv::Point3f n2 = kp2.direction_z;
            cv::Point3f d = kp2.pt3D - kp1.pt3D;
            float d_len = d.dot(d);

            cv::Point3f u = n1;
            cv::Point3f v = d.cross(u);
            cv::Point3f w = u.cross(v);


            float alpha = v.dot(n2);
            float beta = u.dot(d);
            float gamma = atan2(
                    w.dot(n2) / (cv::norm(w) * cv::norm(n2)),
                    u.dot(n2) / (cv::norm(u) * cv::norm(n2))
                    );

            float alpha_c = acos(alpha / (cv::norm(v) * cv::norm(n2)));
            float beta_c = acos(beta / (cv::norm(u) * cv::norm(d)));
            float gamma_c = gamma;

            val[0] = alpha_c * (180 / M_PI);
            val[1] = beta_c * (180 / M_PI);
            val[2] = gamma_c * (180 / M_PI);

            *results = val;
        }

        std::string
        DFunctionFPFH::buildStringImpl() {
            return "FPFH";
        }
    }
}

