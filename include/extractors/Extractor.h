/* 
 * File:   Extractor.h
 * Author: daniele
 *
 * Created on 9 marzo 2015, 18.11
 */

#ifndef EXTRACTOR_H
#define	EXTRACTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boldlib.h>

#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/intersections.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/centroid.h>

#include "KeyPoint3D.h"

namespace visy {
    namespace extractors {

        class Extractor {
        public:
            Extractor();
            Extractor(const Extractor& orig);
            virtual ~Extractor();
            virtual void extract(cv::Mat& source,pcl::PointCloud<PointType>::Ptr cloud, std::vector<KeyPoint3D>& keypoints, cv::Mat* mask = NULL) = 0;
        private:

        };
    }
}

#endif	/* EXTRACTOR_H */

