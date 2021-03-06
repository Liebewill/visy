/*
 * File:   tools.h
 * Author: daniele
 *
 * Created on 23 novembre 2014, 13.09
 */

#ifndef TOOLS_H
#define	TOOLS_H

#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>


#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/registration/icp.h>
#include <pcl/recognition/hv/hv_go.h>

#include "lsd.h"
#include "commons.h"
#include "extractors/KeyPoint3D.h"

namespace visy {
    namespace tools {


        static const int VISY_TOOLS_EDGEDETECTION_METHOD_LSD = 100;
        static const int VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD = 101;

        /**
         * Distance from Point to Line
         * @param v Line first point
         * @param w Line second point
         * @param p Target point
         * @return distance
         */
        float point2LineDistance(cv::Point v, cv::Point w, cv::Point p);


        /**
         * Edge Detection
         * @param source Source image
         * @param lines OUT vector of lines
         * @param method EDGE_DETECTION_METHOD
         */
        void edgeDetection(cv::Mat& source, std::vector<cv::Vec4f>& lines, int method = VISY_TOOLS_EDGEDETECTION_METHOD_LSD);

        /**
         * Simple LSD detection
         * @param gray
         * @param lines
         */
        void edgeDetectionLSD(cv::Mat& gray, std::vector<cv::Vec4f>& lines);

        /**
         * BOLD LSD with refinement and filtering
         * @param gray
         * @param lines
         */
        void edgeDetectionBoldLSD(cv::Mat& gray, std::vector<cv::Vec4f>& lines);

        /**
         * Gets PlaneCoefficient of Cloud
         * @param cloud target Cloud
         * @param indices filter indices
         * @param coefficients OUT coefficients
         * @param inliers OUT ransac inliers
         * @param distance_th segmenation distance th
         */
        void planeCoefficients(pcl::PointCloud<PointType>::Ptr cloud, std::vector<int>& indices, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, float distance_th = 0.01);

        /**
         * Gets Plane Normale Vector of Cloud
         * @param cloud target Cloud
         * @param indices filter indices
         * @param normal_vector OUT normal vector
         * @param distance_th segmentation distance th
         */
        void planeNormalVector(pcl::PointCloud<PointType>::Ptr cloud, std::vector<int>& indices, Eigen::Vector3f& normal_vector, float distance_th = 0.01, bool toward_camera = true);


        /**
         * Pin value in Bin
         * @param range Full search range
         * @param n_bins Number of bins
         * @param value target value
         * @param bin_1 OUT index of bin 1
         * @param bin_2 OUT index of bin 2
         * @param weight_1 OUT weight in bin 1
         * @param weight_2 OUT weight in bin 2
         */
        void pinBin(float& range, int& n_bins, float& value, int& bin_1, int& bin_2, float& weight_1, float& weight_2);


        const int VISY_TOOLS_MATCHING_FULL = 100;
        const int VISY_TOOLS_MATCHING_RATIO = 101;

        /**
         * Match of descriptors
         * @param matches Full matches
         * @param good_matches Filtered matches
         * @param model_descriptor Model Descriptor Mat
         * @param scene_descriptor Scene Descriptor Mat
         * @param match_type Filtering type
         */
        void matchKeypointsBrute(std::vector<cv::DMatch>& matches, std::vector<cv::DMatch>& good_matches, cv::Mat& model_descriptor, cv::Mat& scene_descriptor, int match_type = VISY_TOOLS_MATCHING_FULL);

        /**
         * Cloud Downsampling
         * @param cloud
         * @param cloud_filtered
         * @param leaf
         */
        void cloudDownsample(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_filtered, float leaf);

        /**
         * ICP on instances
         * @param instances
         * @param registered_instances
         * @param max_iterations
         * @param max_distance
         */
        void registerInstances(pcl::PointCloud<PointType>::Ptr reference_cloud, std::vector<pcl::PointCloud<PointType>::ConstPtr>& instances, std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances,std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& registered_transforms, int max_iterations = 5, float max_distance = 0.005f);

        /**
         * GO Hypotheses Verification 
         * @param reference_cloud
         * @param registered_instances
         * @param hypotheses_mask
         * @param inlier_threshold
         */    
        void hypothesesVerification(pcl::PointCloud<PointType>::Ptr reference_cloud, std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances,std::vector<bool>& hypotheses_mask,float inlier_threshold = 0.015f,float occlusion_threshold = 0.01f,float regularizer = 3.0f,float radius_clutter=0.03f,float clutter_regularizer = 5.0f,bool detect_clutter = true,float radius_normal = 0.05f);
        
        
        /**
         * RGB image from registered CLOUD
         * @param cloud input cloud
         * @param image OUT rgb image
         */
        void rgbFromCloud(pcl::PointCloud<PointType>::Ptr cloud, cv::Mat& image);

        void rgbFromCloud(pcl::PointCloud<PointType>::Ptr cloud, cv::Mat& image, std::vector<int>& indices);

        void convertPoint3D(PointType &pt, Eigen::Vector3f &p, bool reverse = false);

        void draw3DVector(pcl::visualization::PCLVisualizer &viewer, Eigen::Vector3f start, Eigen::Vector3f end, float r, float g, float b, std::string name);

        void displayCloud(pcl::visualization::PCLVisualizer &viewer, pcl::PointCloud<PointType>::Ptr cloud, int r, int g, int b, int size, std::string name);

        void displayCloud(pcl::visualization::PCLVisualizer &viewer, pcl::PointCloud<PointType>::ConstPtr cloud, int r, int g, int b, int size, std::string name);

        void display4DHistogram(pcl::visualization::PCLVisualizer& viewer, std::string name, float* histogram, int size, int& viewport);

        Eigen::Matrix4f invertTransformationMatrix(Eigen::Matrix4f& t);

        Eigen::Matrix4f rotationMatrixFromTransformationMatrix(Eigen::Matrix4f& t);

        void poseError(Eigen::Matrix4f& pose1, Eigen::Matrix4f& pose2, float& rotation_error, float& distance_error);

        void transformVector(Eigen::Vector3f& vin, Eigen::Vector3f& vout, Eigen::Matrix4f& transform);

        void transformVector(cv::Point3f& vin, cv::Point3f& vout, Eigen::Matrix4f& transform);

        cv::Scalar avgColor(cv::Mat& source, cv::Point2i center, float radius);


    }
}


#endif	/* TOOLS_H */
