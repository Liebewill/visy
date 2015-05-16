/* 
 * File:   DFunctionBD.h
 * Author: daniele
 *
 * Created on May 16, 2015, 12:58 AM
 */

#ifndef DFUNCTIONBD_H
#define	DFUNCTIONBD_H

#include "DFunction.h"


namespace visy {
    namespace descriptors {

        class DFunctionBD : public DFunction {
        public:
            DFunctionBD(int n_bins, bool multi = false);
            virtual ~DFunctionBD();
            virtual void f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results);
            virtual std::string buildStringImpl();
        private:

        };
    }
}
#endif	/* DFUNCTIONBD_H */

