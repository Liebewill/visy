/* 
 * File:   DFunctionB2D.h
 * Author: daniele
 *
 * Created on 25 marzo 2015, 11.43
 */

#ifndef DFUNCTIONB2D_H
#define	DFUNCTIONB2D_H

#include "DFunction.h"


namespace visy {
    namespace descriptors {

        class DFunctionB2D : public DFunction {
        public:
            DFunctionB2D(int n_bins);
            virtual ~DFunctionB2D();
            virtual void f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results);
            virtual std::string buildStringImpl();
        private:

        };
    }
}
#endif	/* DFUNCTIONB2D_H */

