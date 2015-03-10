/* 
 * File:   Descriptor.h
 * Author: daniele
 *
 * Created on 10 marzo 2015, 18.40
 */

#ifndef DESCRIPTOR_H
#define	DESCRIPTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "extractors/KeyPoint3D.h"


namespace visy {
    namespace descriptors {

        class Descriptor {
        public:
            Descriptor();
            virtual ~Descriptor();
            virtual void describe(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor) = 0;
            virtual void pairKeyPoint3D(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results) = 0;
            
        protected:
            std::string name;
            int size;
        };

    }
}
#endif	/* DESCRIPTOR_H */

