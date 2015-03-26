/* 
 * File:   detectors_utils.h
 * Author: daniele
 *
 * Created on 20 marzo 2015, 22.19
 */

#ifndef DETECTORS_UTILS_H
#define	DETECTORS_UTILS_H

#include "Detector.h"

namespace visy {
    namespace detectors {
        namespace utils {

            /**
             * 
             * @param detector_name
             * @param parameters
             * @return 
             */
            Detector * buildDetectorFromString(std::string detector_name, visy::Parameters* parameters,bool is_model_detector = false);
        }
    }
}



#endif	/* DETECTORS_UTILS_H */

