/* 
 * File:   Utils.h
 * Author: daniele
 *
 * Created on 9 marzo 2015, 21.43
 */

#ifndef UTILS_H
#define	UTILS_H

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
            static void extractSliceAreaFromKeypoint3D( KeyPoint3D& kp, cv::Size2i source_size, std::vector<int>& area, float radius, float slice);
            
            /**
             * Extracts a couple of sliced semicircle areas of indices around target KeyPoint3D
             * @param kp target Keypoint
             * @param source_size reference size (source image size)
             * @param area_left left semicircle area indices
             * @param area_right right semicircle area indices
             * @param radius full area radius
             * @param slice full area slice distance
             */
            static void extractSliceAreaPairFromKeypoint3D( KeyPoint3D& kp, cv::Size2i source_size, std::vector<int>& area_left,std::vector<int>& area_right, float radius, float slice);
            
        }

    }
}

#endif	/* UTILS_H */

