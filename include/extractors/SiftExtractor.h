/* 
 * File:   SiftExtractor.h
 * Author: daniele
 *
 * Created on 21 marzo 2015, 22.58
 */

#ifndef SIFTEXTRACTOR_H
#define	SIFTEXTRACTOR_H

#include "Extractor.h"

namespace visy {
    namespace extractors {

        class SiftExtractor : public Extractor {
        public:
            SiftExtractor();
            virtual ~SiftExtractor();

            virtual void extract(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<KeyPoint3D>& keypoints, cv::Mat* mask = NULL);
            virtual std::string buildNameImpl();
        private:

        };
    }
}
#endif	/* SIFTEXTRACTOR_H */

