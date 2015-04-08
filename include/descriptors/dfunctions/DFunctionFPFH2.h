/* 
 * File:   DFunctionFPFH2.h
 * Author: daniele
 *
 * Created on April 8, 2015, 3:37 PM
 */

#ifndef DFUNCTIONFPFH2_H
#define	DFUNCTIONFPFH2_H

#include "DFunction.h"


namespace visy {
    namespace descriptors {

        class DFunctionFPFH2 : public DFunction {
        public:
            DFunctionFPFH2(int n_bins, bool multi = false);
            virtual ~DFunctionFPFH2();
            virtual void f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results);
            virtual std::string buildStringImpl();
        private:

        };
    }
}
#endif	/* DFUNCTIONFPFH2_H */

