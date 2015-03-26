/* 
 * File:   Bold3DXDetector.h
 * Author: daniele
 *
 * Created on 20 marzo 2015, 22.39
 */

#ifndef BOLD3DXDETECTOR_H
#define	BOLD3DXDETECTOR_H

#include "Detector.h"

namespace visy {
    namespace detectors {

        class Bold3DXDetector : public Detector {
        public:
            Bold3DXDetector(std::vector<float>& sizes, int n_bins = 12, bool filter_occlusion = true, float zone_radius = 5.0f, float zone_slice = 2.0f, float area_normals_angular_th = 25.0f, float area_max_distance = 0.001f);
            virtual void detect(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask = NULL);
            virtual ~Bold3DXDetector();
            virtual std::string buildNameImpl();
//            virtual void refineKeyPoints3D(std::vector<visy::extractors::KeyPoint3D>& keypoints_in, std::vector<visy::extractors::KeyPoint3D>& keypoints_out);
            
        private:

        };
    }
}
#endif	/* BOLD3DXDETECTOR_H */

