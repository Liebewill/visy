/* 
 * File:   HybridDetector.cpp
 * Author: daniele
 * 
 * Created on April 2, 2015, 3:14 PM
 */

#include "HybridDetector.h"
namespace visy {
    namespace detectors {

        HybridDetector::HybridDetector(std::string name, visy::extractors::Extractor* extractor, visy::descriptors::Descriptor* descriptor) : Detector() {
            this->extractor = extractor;
            this->descriptor = descriptor;
            this->hybrid_name = name;
        }

        HybridDetector::~HybridDetector() {
        }

        void
        HybridDetector::detect(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask) {
            std::vector<visy::extractors::KeyPoint3D> temp_keypoints;
            this->extractor->extract(source, cloud, temp_keypoints, mask);
            this->descriptor->describe(source, cloud, temp_keypoints, descriptor);
            visy::extractors::utils::replicateKeypoints(temp_keypoints, keypoints, this->multiplication_factor);
        }

        std::string
        HybridDetector::buildNameImpl() {
            return this->hybrid_name;
        }
    }
}

