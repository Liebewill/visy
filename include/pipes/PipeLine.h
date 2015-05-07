/* 
 * File:   PipeLine.h
 * Author: daniele
 *
 * Created on April 24, 2015, 12:57 PM
 */

#ifndef PIPELINE_H
#define	PIPELINE_H

#include "Detector.h"
#include "KeyPoint3D.h"
#include "extrators_utils.h"

namespace visy {
    namespace pipes {

        struct PipeParameters {
            float gc_size = 0.01f;
            float gc_th = 3;
            //
            std::string downsampling_method = "uniform";
            float downsampling_leaf = 0.005f;
            //
            int icp_max_iterations = 10;
            float icp_max_distance = 0.005f;
            //
            float hv_inlier_threshold = 0.015f;
            float hv_occlusion_threshold = 0.01f;
            float hv_regularizer = 3.0f;
            float hv_radius_clutter = 0.03f;
            float hv_clutter_regularizer = 5.0f;
            bool hv_detect_clutter = true;
            float hv_radius_normal = 0.05f;
        };

        class PipeLine {
        public:
            PipeLine(visy::detectors::Detector* detector, PipeParameters pipeParameters = PipeParameters());
            virtual ~PipeLine();
            void train(
                    pcl::PointCloud<PointType>::Ptr model_cloud,
                    pcl::PointCloud<PointType>::Ptr scene_cloud,
                    std::vector<visy::extractors::KeyPoint3D>& model_keypoints,
                    std::vector<visy::extractors::KeyPoint3D>& scene_keypoints,
                    cv::Mat& model_descriptor,
                    cv::Mat& scene_descriptor,
                    pcl::PointCloud<PointType>::Ptr model_cloud_filtered,
                    pcl::PointCloud<PointType>::Ptr scene_cloud_filtered
                    );

            void run(
                    std::vector<pcl::PointCloud<PointType>::ConstPtr>& instances,
                    std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances,
                    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& transforms,
                    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& registered_transforms,
                    std::vector<bool>& hypotheses_mask
                    );

            PipeParameters pipeParameters;
            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms;
            visy::detectors::Detector* detector;
            pcl::PointCloud<PointType>::Ptr model_cloud;
            pcl::PointCloud<PointType>::Ptr model_cloud_filtered;
            pcl::PointCloud<PointType>::Ptr scene_cloud;
            pcl::PointCloud<PointType>::Ptr scene_cloud_filtered;
        private:






        };
    }
}
#endif	/* PIPELINE_H */

