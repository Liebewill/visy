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


        }
    }
}


#endif	/* DESCRIPTORS_UTILS_H */

