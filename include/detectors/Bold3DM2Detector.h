/* 
 * File:   Bold3DM2Detector.h
 * Author: daniele
 *
 * Created on 11 marzo 2015, 17.30
 */

#ifndef BOLD3DM2DETECTOR_H
#define	BOLD3DM2DETECTOR_H

#include "Detector.h"


namespace visy {
    namespace detectors {

        class Bold3DM2Detector : public Detector {
        public:
            Bold3DM2Detector(std::vector<float>& sizes, int n_bins = 12, bool filter_occlusion = true, float zone_radius = 5.0f, float zone_slice = 2.0f, float area_normals_angular_th = 25.0f, float area_max_distance = 0.001f);
            virtual void detect(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor,cv::Mat* mask = NULL);
//            virtual void refineKeyPoints3D(std::vector<visy::extractors::KeyPoint3D>& keypoints_in,cv::Mat& descriptor_in, std::vector<visy::extractors::KeyPoint3D>& keypoints_out,cv::Mat& descriptor_out);
            virtual ~Bold3DM2Detector();
            virtual std::string buildNameImpl();

        private:

        };
    }
}
#endif	/* BOLD3DM2DETECTOR_H */

