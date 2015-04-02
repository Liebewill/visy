/* 
 * File:   Extractor.h
 * Author: daniele
 *
 * Created on 9 marzo 2015, 18.11
 */

#ifndef EXTRACTOR_H
#define	EXTRACTOR_H

#include <opencv2/opencv.hpp>

#include <boldlib.h>

#include <pcl/point_cloud.h>

#include "KeyPoint3D.h"

namespace visy {
    namespace extractors {
        
        class Extractor {
        public:
            
            Extractor();
            Extractor(const Extractor& orig);
            virtual ~Extractor();
            virtual void extract(cv::Mat& source,pcl::PointCloud<PointType>::Ptr cloud, std::vector<KeyPoint3D>& keypoints, cv::Mat* mask = NULL) = 0;
            std::string buildName() ;
            virtual  std::string buildNameImpl() = 0;
        protected:
            std::string name;

        };
    }
}

#endif	/* EXTRACTOR_H */

