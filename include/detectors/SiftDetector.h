/* 
 * File:   SiftDetector.h
 * Author: daniele
 *
 * Created on May 12, 2015, 11:43 AM
 */

#ifndef SIFTDETECTOR_H
#define	SIFTDETECTOR_H

#include "Detector.h"

namespace visy {
    namespace detectors {

        class SiftDetector : public Detector {
        public:
            SiftDetector();
           
            virtual ~SiftDetector();
            
             virtual void detect(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask = NULL);
            
            virtual std::string buildNameImpl();
            
        private:

        };
    }
}
#endif	/* SIFTDETECTOR_H */

