/* 
 * File:   KeyPoint3D.h
 * Author: daniele
 *
 * Created on 9 marzo 2015, 18.03
 */

#ifndef KEYPOINT3D_H
#define	KEYPOINT3D_H

#include <iostream>
#include <vector>
#include <math.h>

#include <opencv2/opencv.hpp>

#include <boldlib.h>

#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include "tools.h"
#include "commons.h"

namespace visy {
    namespace extractors {

        

        class KeyPoint3D : public cv::KeyPoint {
        public:


            static const int KEYPOINT3D_TYPE_UNKNOWN = 0;
            static const int KEYPOINT3D_TYPE_INVALID = 666;
            static const int KEYPOINT3D_TYPE_EDGE_SURFACE = 1000;
            static const int KEYPOINT3D_TYPE_EDGE_TEXTURE = 1001;
            static const int KEYPOINT3D_TYPE_EDGE_OCCLUSION = 1002;
            static const int KEYPOINT3D_TYPE_EDGE_OCCLUSION_EXT = 1003;
            static const int KEYPOINT3D_TYPE_2D_SURFACE = 1004;
            static const int KEYPOINT3D_TYPE_2D_TEXTURE = 1005;
            static const int KEYPOINT3D_TYPE_2D_OCCLUSION = 1006;

            static int STATIC_COUNTER;

            KeyPoint3D(cv::Vec4f line, pcl::PointCloud<PointType>::Ptr cloud);
            KeyPoint3D(cv::KeyPoint kp, pcl::PointCloud<PointType>::Ptr cloud);
            KeyPoint3D();

            virtual ~KeyPoint3D();

            static void draw3DKeyPoints(cv::Mat& out, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Scalar color, float tick, bool force_color = false);
            static void draw3DKeyPointsWithAreas(cv::Mat& out, std::vector<KeyPoint3D>& keypoints, cv::Scalar color, float tick, float radius, float slice);
            static void draw3DKeyPoints3D(pcl::visualization::PCLVisualizer &viewer, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Scalar color, std::string name, bool simple = false);

            cv::Point2i pt1;
            cv::Point2i pt2;

            cv::Point3f pt3D;
            cv::Point3f pt3D_1;
            cv::Point3f pt3D_2;
            cv::Point3f direction_x;
            cv::Point3f direction_y;
            cv::Point3f direction_z;

            RFType reference_frame;
            
            int type;

            KeyPoint3D clone();
            KeyPoint3D cloneTranslated(pcl::PointCloud<PointType>::Ptr cloud, cv::Point2f& translation_2d);

            static void transformKeyPoint3D(KeyPoint3D& in, KeyPoint3D& out, Eigen::Matrix4f& transform);
            static void transformKeyPoint3Ds(std::vector<KeyPoint3D>& keypoint_in, std::vector<KeyPoint3D>& keypoints_out, Eigen::Matrix4f& transform);
        protected:

        };
        
        struct KeyPoint3DZOrderer {

            inline bool operator()(const KeyPoint3D& kp1, const KeyPoint3D& kp2) {
                return kp1.pt3D.z < kp2.pt3D.z;
            }
        };
    }
}
#endif	/* KEYPOINT3D_H */

