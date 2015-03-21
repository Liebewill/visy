/* 
 * File:   DFunctionB3DV2.h
 * Author: daniele
 *
 * Created on 15 marzo 2015, 13.54
 */

#ifndef DFUNCTIONB3DV2_H
#define	DFUNCTIONB3DV2_H

#include "DFunction.h"

namespace visy {
    namespace descriptors {

        class DFunctionB3DV2 : public DFunction {
        public:
            DFunctionB3DV2(int n_bins);
            virtual ~DFunctionB3DV2();
            virtual void f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results);
            virtual std::string buildStringImpl();
        private:

        };
    }
}
#endif	/* DFUNCTIONB3DV2_H */

