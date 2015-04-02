/* 
 * File:   DFunctionB3D4H.h
 * Author: daniele
 *
 * Created on April 2, 2015, 4:57 PM
 */

#ifndef DFUNCTIONB3D4H_H
#define	DFUNCTIONB3D4H_H

#include "DFunction.h"


namespace visy {
    namespace descriptors {

        class DFunctionB3D4H : public DFunction {
        public:
            DFunctionB3D4H(int n_bins);
            virtual ~DFunctionB3D4H();
            virtual void f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results);
            virtual std::string buildStringImpl();
        private:

        };
    }
}

#endif	/* DFUNCTIONB3D4H_H */

