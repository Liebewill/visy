/* 
 * File:   Bold3DR2MultiDetector.h
 * Author: daniele
 *
 * Created on 21 marzo 2015, 22.09
 */

#ifndef BOLD3DR2MULTIDETECTOR_H
#define	BOLD3DR2MULTIDETECTOR_H

#include "Detector.h"

namespace visy {
    namespace detectors {

        class Bold3DR2MultiDetector : public Detector {
        public:
            Bold3DR2MultiDetector(std::vector<float>& radiuses, int n_bins = 12, bool filter_occlusion = true, float zone_radius = 5.0f, float zone_slice = 2.0f, float area_normals_angular_th = 25.0f, float area_max_distance = 0.001f);
            virtual void detect(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask = NULL);
            
            virtual ~Bold3DR2MultiDetector();
            virtual std::string buildNameImpl();
        private:

        };
    }
}
#endif	/* BOLD3DR2MULTIDETECTOR_H */

