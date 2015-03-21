/* 
 * File:   HistogramND.h
 * Author: daniele
 *
 * Created on 20 marzo 2015, 17.44
 */

#ifndef HISTOGRAMND_H
#define	HISTOGRAMND_H

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <opencv2/core/core.hpp>

namespace visy {
    namespace descriptors {

        class HistogramND {
        public:
            HistogramND(int dimension, float range, int n_bins);
            virtual ~HistogramND();

            void placeValue(float target_value, int dim, float& center, int* idxs, float* weights);
            void placeVector(float* target_values, float weight = 1.0f);
            float normalizedValue(float value, int dim);
            int binIndex(float target_value, int dim);
            float binCenter(float target_value, int dim);
            void pin(int x, int y, int z, float value);

            float theta[3];
            float max[3];
            int size[3];

            float theta_w;
            float theta_h;
            float theta_d;

            float* data;
            
            int getSize();
            void normalize();
            void insertInMat (cv::Mat& out, int row);
            void reset();
        private:
            int dimension;
            int n_bins;
            int full_size;
            float range;
            void build(float theta_w, float theta_h, float theta_d, int width, int height, int depth);
            
            int pin_counter;
        };
    }
}
#endif	/* HISTOGRAMND_H */

