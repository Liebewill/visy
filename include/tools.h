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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/intersections.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/centroid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>

#include "lsd.h"

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::Label LabelType;


namespace visy {
    namespace tools {


        static const int VISY_TOOLS_EDGEDETECTION_METHOD_LSD = 100;
        
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
         * Gets PlaneCoefficient of Cloud
         * @param cloud target Cloud
         * @param indices filter indices
         * @param coefficients OUT coefficients
         * @param inliers OUT ransac inliers
         * @param distance_th segmenation distance th
         */
        void planeCoefficients(pcl::PointCloud<PointType>::Ptr cloud,std::vector<int>& indices,pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers,float distance_th =0.01);
        
        /**
         * RGB image from registered CLOUD
         * @param cloud input cloud
         * @param image OUT rgb image
         */
        void rgbFromCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, cv::Mat& image);
        
        void rgbFromCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, cv::Mat& image, std::vector<int>& indices);

        void convertPoint3D(PointType &pt, Eigen::Vector3f &p, bool reverse = false);

        void draw3DVector(pcl::visualization::PCLVisualizer &viewer, Eigen::Vector3f start, Eigen::Vector3f end, float r, float g, float b, std::string name);

        void displayCloud(pcl::visualization::PCLVisualizer &viewer, pcl::PointCloud<PointType>::Ptr cloud, int r, int g, int b, int size, std::string name);

        void display4DHistogram(pcl::visualization::PCLVisualizer& viewer, std::string name, float* histogram, int size, int& viewport);

        Eigen::Matrix4f invertTransformationMatrix(Eigen::Matrix4f& t);

        void poseError(Eigen::Matrix4f& pose1, Eigen::Matrix4f& pose2, float& rotation_error, float& distance_error);

        void transformVector(Eigen::Vector3f& vin, Eigen::Vector3f& vout, Eigen::Matrix4f& transform);

        cv::Scalar avgColor(cv::Mat& source, cv::Point2i center, float radius);
    }
}


#endif	/* TOOLS_H */
