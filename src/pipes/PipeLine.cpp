/* 
 * File:   PipeLine.cpp
 * Author: daniele
 * 
 * Created on April 24, 2015, 12:57 PM
 */

#include <vector>
#include <pcl/common/io.h>

#include "PipeLine.h"

namespace visy {
    namespace pipes {

        PipeLine::PipeLine(visy::detectors::Detector* detector, PipeParameters pipeParameters) {
            this->detector = detector;
            this->pipeParameters = pipeParameters;
        }

        PipeLine::~PipeLine() {
        }

        void PipeLine::train(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scene_cloud, std::vector<visy::extractors::KeyPoint3D>& model_keypoints, std::vector<visy::extractors::KeyPoint3D>& scene_keypoints, cv::Mat& model_descriptor, cv::Mat& scene_descriptor, pcl::PointCloud<PointType>::Ptr model_cloud_filtered, pcl::PointCloud<PointType>::Ptr scene_cloud_filtered) {
            this->model_cloud_filtered = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
            this->scene_cloud_filtered = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
            
            visy::extractors::utils::modelSceneMatch(
                    model_keypoints,
                    scene_keypoints,
                    model_descriptor,
                    scene_descriptor,
                    this->transforms,
                    this->pipeParameters.gc_size,
                    this->pipeParameters.gc_th
                    );

            visy::tools::cloudDownsample(model_cloud, this->model_cloud_filtered, this->pipeParameters.downsampling_leaf);
            visy::tools::cloudDownsample(scene_cloud, this->scene_cloud_filtered, this->pipeParameters.downsampling_leaf);
            pcl::copyPointCloud(*this->model_cloud_filtered,*model_cloud_filtered);
            pcl::copyPointCloud(*this->scene_cloud_filtered,*scene_cloud_filtered);
            
        }

        void PipeLine::run(std::vector<pcl::PointCloud<PointType>::ConstPtr>& instances, std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& transforms, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& registered_transforms, std::vector<bool>& hypotheses_mask) {
            
            transforms.insert(transforms.end(),this->transforms.begin(),this->transforms.end());
            /**
             * Generates clouds for each instances found 
             */
            for (size_t i = 0; i < this->transforms.size(); ++i) {
                pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType> ());
                pcl::transformPointCloud(*model_cloud_filtered, *rotated_model, this->transforms.at(i));
                instances.push_back(rotated_model);
            }

            /**
             * ICP
             */
            visy::tools::registerInstances(
                    scene_cloud_filtered,
                    instances,
                    registered_instances,
                    registered_transforms,
                    this->pipeParameters.icp_max_iterations,
                    this->pipeParameters.icp_max_distance);
            
//            registered_instances = instances;
            /**
             * Hypothesis Verification
             */
            visy::tools::hypothesesVerification(
                    scene_cloud_filtered,
                    registered_instances,
                    hypotheses_mask,
                    this->pipeParameters.hv_inlier_threshold,
                    this->pipeParameters.hv_occlusion_threshold,
                    this->pipeParameters.hv_regularizer,
                    this->pipeParameters.hv_radius_clutter,
                    this->pipeParameters.hv_clutter_regularizer,
                    this->pipeParameters.hv_detect_clutter,
                    this->pipeParameters.hv_radius_normal
                    );

        }


    }
}