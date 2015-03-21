/* 
 * File:   Bold3DM2MultiDetector.h
 * Author: daniele
 *
 * Created on 20 marzo 2015, 18.19
 */

#ifndef BOLD3DM2MULTIDETECTOR_H
#define	BOLD3DM2MULTIDETECTOR_H

#include "Detector.h"


namespace visy {
    namespace detectors {

        class Bold3DM2MultiDetector : public Detector {
        public:
            Bold3DM2MultiDetector(std::vector<float>& sizes, int n_bins = 12, bool filter_occlusion = true, float zone_radius = 5.0f, float zone_slice = 2.0f, float area_normals_angular_th = 25.0f, float area_max_distance = 0.001f);
            virtual void detect(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask = NULL);

            virtual ~Bold3DM2MultiDetector();
            virtual std::string buildNameImpl();
        private:

        };
    }
}

#endif	/* BOLD3DM2MULTIDETECTOR_H */

