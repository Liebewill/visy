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


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boldlib.h>

#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/intersections.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/centroid.h>

#include "tools.h"

namespace visy {
    namespace extractors {

        class KeyPoint3D : public cv::KeyPoint {
        public:
            KeyPoint3D(cv::Vec4f line, pcl::PointCloud<PointType>::Ptr cloud);
            virtual ~KeyPoint3D();
            static void draw3DKeyPoints(cv::Mat& out, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Scalar color, float tick);
            static void draw3DKeyPointsWithAreas(cv::Mat& out, std::vector<KeyPoint3D>& keypoints, cv::Scalar color, float tick,float radius, float slice);
            static void draw3DKeyPoints3D(pcl::visualization::PCLVisualizer &viewer, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Scalar color,std::string name);
            
            void bindArea(cv::Size2i source_size,std::vector<int>& area,float radius, float slice);
            void bindAreas(cv::Size2i source_size,std::vector<int>& area_left,std::vector<int>& area_right,float radius, float slice);
            
            cv::Point2i pt1;
            cv::Point2i pt2;
            
            cv::Point3f pt3D;
            cv::Point3f pt3D_1;
            cv::Point3f pt3D_2;
            cv::Point3f direction;
        protected:

        };
    }
}
#endif	/* KEYPOINT3D_H */

