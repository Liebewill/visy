/* 
 * File:   DFunction.h
 * Author: daniele
 *
 * Created on 15 marzo 2015, 13.49
 */

#ifndef DFUNCTION_H
#define	DFUNCTION_H

#include "extractors/KeyPoint3D.h"
#include "signatures/Signature.h"

namespace visy {
    namespace descriptors {

        class DFunction {
        public:
            DFunction();
            virtual ~DFunction();
            virtual void f(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results) = 0;
            virtual std::string buildStringImpl() = 0;
            virtual std::string buildString();
            int getSize();
            int getDataSize();
            Signature* getSignature();
        protected:
            int size;
            Signature* signature;
            bool sparse;
        };
    }
}
#endif	/* DFUNCTION_H */

