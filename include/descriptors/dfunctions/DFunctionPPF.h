/* 
 * File:   DFunctionPPF.h
 * Author: daniele
 *
 * Created on April 20, 2015, 11:53 AM
 */

#ifndef DFUNCTIONPPF_H
#define	DFUNCTIONPPF_H

#include "DFunction.h"


namespace visy {
    namespace descriptors {

        class DFunctionPPF : public DFunction{
        public:
            DFunctionPPF(int n_bins);
            virtual ~DFunctionPPF();
            virtual void f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results);
            virtual std::string buildStringImpl();
        private:

        };

    }
}
#endif	/* DFUNCTIONPPF_H */

