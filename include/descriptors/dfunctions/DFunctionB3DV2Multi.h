/* 
 * File:   DFunctionB3DV2Multi.h
 * Author: daniele
 *
 * Created on 20 marzo 2015, 17.36
 */

#ifndef DFUNCTIONB3DV2MULTI_H
#define	DFUNCTIONB3DV2MULTI_H

#include "DFunction.h"


namespace visy {
    namespace descriptors {

        class DFunctionB3DV2Multi : public DFunction {
        public:
            DFunctionB3DV2Multi(int n_bins);
            virtual ~DFunctionB3DV2Multi();
            virtual void f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results);
            virtual std::string buildStringImpl();
        private:

        };
    }
}
#endif	/* DFUNCTIONB3DV2MULTI_H */

