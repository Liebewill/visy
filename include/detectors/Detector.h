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
#include <commons/Parameters.h>

namespace visy {
    namespace detectors {

        class Detector {
        public:
            Detector();
            virtual ~Detector();
            virtual void detect(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask = NULL) = 0;
            virtual void refineKeyPoints3D(std::vector<visy::extractors::KeyPoint3D>& keypoints_in, cv::Mat& descriptor_in, std::vector<visy::extractors::KeyPoint3D>& keypoints_out, cv::Mat& descriptor_out);
            virtual void refineKeyPoints3D(std::vector<visy::extractors::KeyPoint3D>& keypoints_in,  std::vector<visy::extractors::KeyPoint3D>& keypoints_out);
            std::string buildName();
            virtual std::string buildNameImpl() = 0;
            visy::extractors::Extractor * extractor;
            visy::descriptors::Descriptor * descriptor;
            bool isDetectorEmbedded();
            int getMultiplicationFactor();
        protected:
            int multiplication_factor;
            bool detector_embedded;
        };
    }
}
#endif	/* DETECTOR_H */

