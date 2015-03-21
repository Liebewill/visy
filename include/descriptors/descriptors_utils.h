/* 
 * File:   descriptors_utils.h
 * Author: daniele
 *
 * Created on 15 marzo 2015, 13.46
 */

#ifndef DESCRIPTORS_UTILS_H
#define	DESCRIPTORS_UTILS_H

#include "extractors/KeyPoint3D.h"
#include "dfunctions/DFunction.h"
#include <pcl/kdtree/kdtree_flann.h>

namespace visy {
    namespace descriptors {
        namespace utils {

            /**
             * Applies DFunction to Bunch around KeyPoint3D
             * @param kp3d target KeyPoint
             * @param sizes Multi Bunch Sizes
             * @param bunch_method Type of kdtree search
             * @param dfunction Used DFunction
             * @param results OUT results
             */
            void multiBunchDescription(visy::extractors::KeyPoint3D& kp3d,std::vector<visy::extractors::KeyPoint3D>& keypoints, std::vector<float>& sizes, int bunch_method, DFunction* dfunction,std::vector<Signature>& signatures);
            
            /**
             * Applies DFunction to Bunch around KeyPoint3D
             * @param kp3d
             * @param keypoints
             * @param sizes
             * @param bunch_method
             * @param dfunction
             * @param descriptor_out
             * @param row
             */
            void multiBunchDescription(visy::extractors::KeyPoint3D& kp3d,std::vector<visy::extractors::KeyPoint3D>& keypoints,pcl::KdTreeFLANN<PointType>& kdtree, std::vector<float>& sizes, int bunch_method, DFunction* dfunction,cv::Mat& descriptor_out, int row);


        }
    }
}


#endif	/* DESCRIPTORS_UTILS_H */

