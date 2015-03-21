/* 
 * File:   Signature.h
 * Author: daniele
 *
 * Created on 15 marzo 2015, 14.35
 */

#ifndef SIGNATURE_H
#define	SIGNATURE_H

#include "Histogram1D.h"
#include "boost/shared_ptr.hpp"
#include "HistogramND.h"
#include <cmath>

namespace visy {
    namespace descriptors {

        class Signature {
        public:
            static int STATIC_COUNTER;
            Signature(int dimension,float range,int n_bins, bool sparse = false);
            virtual ~Signature();
            float* getData();
            void pinValue(float* values);
            void finalize();
            void insertInMat(cv::Mat& out, int row);
            int getSize();
            void initialize();
        protected:
            int dimension;
            float range;
            int n_bins;
            bool sparse;
            std::vector<boost::shared_ptr<Histogram1D> > histograms;
            boost::shared_ptr<HistogramND> multiHistogram;
            
        };
    }
}
#endif	/* SIGNATURE_H */

