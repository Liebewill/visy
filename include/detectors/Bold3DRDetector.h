/* 
 * File:   Bold3DRDetector.h
 * Author: daniele
 *
 * Created on 11 marzo 2015, 17.52
 */

#ifndef BOLD3DRDETECTOR_H
#define	BOLD3DRDETECTOR_H

#include "Detector.h"


namespace visy {
    namespace detectors {

        class Bold3DRDetector : public Detector {
        public:
            Bold3DRDetector(std::vector<float>& radiuses, int n_bins = 12, bool filter_occlusion = true, float zone_radius = 5.0f, float zone_slice = 2.0f, float area_normals_angular_th = 25.0f, float area_max_distance = 0.001f);
            virtual void detect(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask = NULL);
            virtual ~Bold3DRDetector();
            virtual std::string buildNameImpl();
        private:

        };
    }
}
#endif	/* BOLD3DRDETECTOR_H */

