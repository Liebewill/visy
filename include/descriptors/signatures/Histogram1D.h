/* 
 * File:   Histogram1D.h
 * Author: daniele
 *
 * Created on 10 marzo 2015, 21.06
 */

#ifndef HISTOGRAM1D_H
#define	HISTOGRAM1D_H

#include <ostream>
#include <opencv2/core/core.hpp>
namespace visy {
    namespace descriptors {
        
        
        class Histogram1D {
            
            
        
            
        public:
            static int STATIC_COUNTER;
            Histogram1D(float range,int n_bins);
            virtual ~Histogram1D();
            float* data;
            void pinValue(float value);
            void normalize();
            void concat(Histogram1D& h);
            void insertInMat(cv::Mat& out, int row);
            friend std::ostream& operator<< (std::ostream& stream, const Histogram1D& h);
        private:
            float range;
            int n_bins;
            int pin_counter;
        };
        
    }
}
#endif	/* HISTOGRAM1D_H */

