/* 
 * File:   DFunctionFPFH.h
 * Author: daniele
 *
 * Created on April 2, 2015, 4:04 PM
 */

#ifndef DFUNCTIONFPFH_H
#define	DFUNCTIONFPFH_H

#include "DFunction.h"

namespace visy {
    namespace descriptors {

        class DFunctionFPFH : public DFunction {
        public:
            DFunctionFPFH(int n_bins, bool multi = false);
            virtual ~DFunctionFPFH();
            virtual void f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results);
            virtual std::string buildStringImpl();
        private:

        };
    }
}
#endif	/* DFUNCTIONFPFH_H */

