/* 
 * File:   DFunctionB3DV1Multi.h
 * Author: daniele
 *
 * Created on 20 marzo 2015, 17.40
 */

#ifndef DFUNCTIONB3DV1MULTI_H
#define	DFUNCTIONB3DV1MULTI_H

#include "DFunction.h"


namespace visy {
    namespace descriptors {

        class DFunctionB3DV1Multi : public DFunction{
        public:
            DFunctionB3DV1Multi(int n_bins);
            virtual ~DFunctionB3DV1Multi();
            virtual void f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results);
            virtual std::string buildStringImpl();
        private:

        };
    }
}
#endif	/* DFUNCTIONB3DV1MULTI_H */

