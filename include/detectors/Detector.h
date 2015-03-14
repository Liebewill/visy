/* 
 * File:   Detector.h
 * Author: daniele
 *
 * Created on 11 marzo 2015, 16.44
 */

#ifndef DETECTOR_H
#define	DETECTOR_H

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

#include <extractors/Extractor.h>
#include <descriptors/Descriptor.h>

namespace visy {
    namespace detectors {

        class Detector {
        public:
            Detector();
            virtual ~Detector();
            virtual void detect(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor,cv::Mat* mask = NULL) = 0;
            std::string buildName();
            virtual std::string buildNameImpl() = 0;
        protected:
            visy::extractors::Extractor * extractor;
            visy::descriptors::Descriptor * descriptor;
            int multiplication_factor;
        };
    }
}
#endif	/* DETECTOR_H */
