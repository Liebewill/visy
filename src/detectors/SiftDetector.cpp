/* 
 * File:   SiftDetector.cpp
 * Author: daniele
 * 
 * Created on May 12, 2015, 11:43 AM
 */

#include <opencv2/nonfree/features2d.hpp>

#include "SiftDetector.h"

namespace visy {
    namespace detectors {

        SiftDetector::SiftDetector() {
        }

        SiftDetector::~SiftDetector() {
        }

        void SiftDetector::detect(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask) {
            cv::SiftFeatureDetector detector;

            std::vector<cv::KeyPoint> sift_keypoints;
            
            detector.detect(source,sift_keypoints);
            detector.compute(source, sift_keypoints,descriptor);
            int nkps = sift_keypoints.size();
            
            for (int i = 0; i < nkps; i++) {
                visy::extractors::KeyPoint3D kp3d(sift_keypoints[i],cloud);
                keypoints.push_back(kp3d);
            }

        }

        std::string SiftDetector::buildNameImpl() {
            return "SIFT";
        }

    }
}
