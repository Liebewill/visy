/* 
 * File:   HybridDetector.h
 * Author: daniele
 *
 * Created on April 2, 2015, 3:14 PM
 */

#ifndef HYBRIDDETECTOR_H
#define	HYBRIDDETECTOR_H

#include "Detector.h"

namespace visy {
    namespace detectors {

        class HybridDetector : public Detector {
        public:
            HybridDetector(std::string name, visy::extractors::Extractor* extractor, visy::descriptors::Descriptor* descriptor);
            virtual void detect(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask = NULL);

            virtual ~HybridDetector();
            virtual std::string buildNameImpl();
        protected:
            std::string hybrid_name;

        };
    }
}

#endif	/* HYBRIDDETECTOR_H */

