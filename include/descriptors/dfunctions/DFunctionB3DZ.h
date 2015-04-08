/* 
 * File:   DFunctionB3DZ.h
 * Author: daniele
 *
 * Created on April 7, 2015, 3:45 PM
 */

#ifndef DFUNCTIONB3DZ_H
#define	DFUNCTIONB3DZ_H

#include "DFunction.h"


namespace visy {
    namespace descriptors {

        class DFunctionB3DZ : public DFunction {
        public:
            DFunctionB3DZ(int n_bins, bool multi = false);
            virtual ~DFunctionB3DZ();
            virtual void f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results);
            virtual std::string buildStringImpl();
        private:

        };
    }
}
#endif	/* DFUNCTIONB3DZ_H */

