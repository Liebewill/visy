/* 
 * File:   Utils.h
 * Author: daniele
 *
 * Created on 9 marzo 2015, 21.43
 */

#ifndef EXTRACTORS_UTILS_H
#define	EXTRACTORS_UTILS_H

#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>


#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/intersections.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/recognition/cg/geometric_consistency.h>

#include "KeyPoint3D.h"

namespace visy {
    namespace extractors {
        namespace utils {
            /**
             * Extracts a sliced circle area of indices around target KeyPoint3D
             * @param kp target Keypoint
             * @param source_size reference size (source image size)
             * @param area vector of extracted indices
             * @param radius area radius
             * @param slice slice distance
             */
            void extractSliceAreaFromKeypoint3D(KeyPoint3D& kp, cv::Size2i source_size, std::vector<int>& area, float radius, float slice);

            /**
             * Extracts a couple of sliced semicircle areas of indices around target KeyPoint3D
             * @param kp target Keypoint
             * @param source_size reference size (source image size)
             * @param area_left left semicircle area indices
             * @param area_right right semicircle area indices
             * @param radius full area radius
             * @param slice full area slice distance
             */
            void extractSliceAreaPairFromKeypoint3D(KeyPoint3D& kp, cv::Size2i source_size, std::vector<int>& area_left, std::vector<int>& area_right, float radius, float slice);



            /**
             * Draw KeyPoint3Ds with sliced areas
             * @param out
             * @param keypoints
             * @param color
             * @param tick
             * @param radius
             * @param slice
             */
            void draw3DKeyPointsWithAreas(cv::Mat& out, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Scalar color, float tick, float radius, float slice);


            /**
             * Builds a Cloud of primitive points from Keypoint list
             * @param cloud OUT target cloud
             * @param keypoints source keypoints
             */
            void buildPrimiteCloudFromKeypoints(pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints);

            /**
             * Filters Consensus Set between keypoints
             * @param model_keypoints Model Keypoints
             * @param scene_keypoints Scene Keypoints
             * @param matches Keypoints matches
             * @param matched_model_keypoints OUT Matched Model Keypoints
             * @param matched_scene_keypoints OUT Matched Scene Keypoints
             * @param matched_model_keypoints_indices OUT Matched Model Keypoints indices
             * @param matched_scene_keypoints_indices OUT Matched Scene Keypoints indices
             * @param model_scene_corrs OUT model_scene correspondences
             */
            void
            keypointsConsensusSet(
                    std::vector<visy::extractors::KeyPoint3D >& model_keypoints,
                    std::vector<visy::extractors::KeyPoint3D >& scene_keypoints,
                    std::vector<cv::DMatch>& matches,
                    std::vector<visy::extractors::KeyPoint3D>& matched_model_keypoints,
                    std::vector<visy::extractors::KeyPoint3D>& matched_scene_keypoints,
                    std::vector<int>& matched_model_keypoints_indices,
                    std::vector<int>& matched_scene_keypoints_indices,
                    pcl::CorrespondencesPtr& model_scene_corrs);

            /**
             * Geometry Consensus
             * @param gc_size GC Size
             * @param gc_th GC Threshols
             * @param model_keypoints Model Keypoints
             * @param scene_keypoints Scene Keypoints
             * @param model_scene_corrs OUT Model-Scene Correspondences
             * @param rototranslations OUT Rototraslations found
             * @param clustered_corrs OUT Clustere correspondences found
             */
            void keypointsGeometricConsistencyGrouping(double gc_size, int gc_th,
                    std::vector<visy::extractors::KeyPoint3D>& model_keypoints,
                    std::vector<visy::extractors::KeyPoint3D>& scene_keypoints,
                    pcl::CorrespondencesPtr& model_scene_corrs,
                    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& rototranslations,
                    std::vector < pcl::Correspondences >& clustered_corrs);


            /**
             * Replicates keypoints 
             * @param source source vector
             * @param out OUT target replicated vector :  (1,2,3) -> (1,2,3,1,2,3,1,2,3...)
             * @param times number of replications
             */
            void replicateKeypoints(std::vector<KeyPoint3D>& source, std::vector<KeyPoint3D>& out, int times = 1);



            /**
             * Model Scene Full Match
             * @param model_keypoints
             * @param scene_keypoints
             * @param model_descriptor
             * @param scene_descriptor
             * @param rototranslations
             */
            void modelSceneMatch(
                    std::vector<visy::extractors::KeyPoint3D>& model_keypoints,
                    std::vector<visy::extractors::KeyPoint3D>& scene_keypoints,
                    cv::Mat& model_descriptor,
                    cv::Mat& scene_descriptor,
                    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& rototranslations,
                    float gc_size,
                    float gc_th,
                    int match_type = visy::tools::VISY_TOOLS_MATCHING_FULL);
            
            /**
             * Checks if target KeyPoint3D is in occlusion side wrt source KeyPoint3D
             * @param source
             * @param target
             * @return TRUE or FALSE
             */
            bool isInOcclusionSide(visy::extractors::KeyPoint3D& source,visy::extractors::KeyPoint3D& target);
        }

    }
}

#endif	/* EXTRACTORS_UTILS_H */

