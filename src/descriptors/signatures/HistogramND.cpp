/* 
 * File:   HistogramND.cpp
 * Author: daniele
 * 
 * Created on 20 marzo 2015, 17.44
 */

#include "signatures/HistogramND.h"

namespace visy
{
  namespace descriptors
  {

    HistogramND::HistogramND (int dimension, float range, int n_bins)
    {
      this->pin_counter = 0;
      this->dimension = dimension;
      this->range = range;
      this->n_bins = n_bins;

      float tw = range / n_bins;
      this->build(tw, tw, tw, n_bins, n_bins, n_bins);
    }

    HistogramND::~HistogramND ()
    {
      delete[] this->data;
    }

    void
    HistogramND::reset ()
    {
      this->pin_counter = 0;
      for (int i = 0; i < this->full_size; i++)
      {
        this->data[i] = 0.0f;
      }
    }

    void
    HistogramND::build (float theta_w, float theta_h, float theta_d, int width, int height, int depth)
    {
      this->theta[0] = theta_w;
      this->theta[1] = theta_h;
      this->theta[2] = theta_d;
      this->size[0] = width;
      this->size[1] = height;
      this->size[2] = depth;

      for (int i = 0; i < 3; i++)
      {
        max[i] = this->theta[i] * this->size[i];
      }

      const int fullSize = this->size[0] * this->size[1] * this->size[2];
      this->data = new float[fullSize];
      std::fill_n(this->data, fullSize, 0.0f);
      this->full_size = fullSize;
    }

    float
    HistogramND::binCenter (float target_value, int dim)
    {
      float value = this->normalizedValue(target_value, dim);
      int index = this->binIndex(value, dim);
      float center = index * this->theta[dim] + this->theta[dim] / 2.0f;
      return center;
    }

    int
    HistogramND::binIndex (float target_value, int dim)
    {
      float value = this->normalizedValue(target_value, dim);
      float cl = floor(value / this->theta[dim]) * this->theta[dim];
      cl = cl / this->theta[dim];
      int index = (int) cl;
      return index;
    }

    float
    HistogramND::normalizedValue (float value, int dim)
    {
      while (value > max[dim])
      {
        value -= max[dim];
      }
      return value;
    }

    void
    HistogramND::pin (int x, int y, int z, float value)
    {
      int index = x + y * this->size[0] + z * this->size[0] * this->size[1];
      if (index >= 0 && index < this->getSize())
      {
        if (value != value)
        {
          value = 0.0f;
        }
        this->data[index] += value;
      }
    }

    int
    HistogramND::getSize ()
    {
      return this->size[0] * this->size[1] * this->size[2];
    }

    void
    HistogramND::placeValue (float target_value, int dim, float& center, int* idxs, float* weights)
    {
      float value = this->normalizedValue(target_value, dim);
      float cl = floor(value / this->theta[dim]) * this->theta[dim];
      cl = cl / this->theta[dim];
      cl = cl == this->size[dim] ? 0 : cl;
      idxs[0] = (int) cl;
      center = idxs[0] * this->theta[dim] + this->theta[dim] / 2.0f;
      float delta = value - center;
      if (delta >= 0)
      {
        idxs[1] = idxs[0] + 1;
        if (idxs[1] >= this->size[dim])
        {
          idxs[1] = 0;
        }
      }
      else
      {
        idxs[1] = idxs[0] - 1;
        if (idxs[1] < 0)
        {
          idxs[1] = this->size[dim] - 1;
        }
      }
      weights[0] = 1 - std::abs(delta) / this->theta[dim];
      weights[1] = 1 - weights[0];
    }

    void
    HistogramND::placeVector (float* target_values, float weight)
    {
      float* values = new float[3];
      float* centers = new float[3];
      int** idx = new int*[3];
      float** w = new float*[3];
      for (int i = 0; i < 3; i++)
      {
        values[i] = this->normalizedValue(target_values[i], i);
        idx[i] = new int[2];
        w[i] = new float[2];
        this->placeValue(values[i], i, centers[i], idx[i], w[i]);
        //            std::cout << "Value(" << i << ") " << values[i] << " center:" << centers[i] << " : "
        //                    << idx[i][0] << "," << idx[i][1] << ","
        //                    << w[i][0] << "," << w[i][1] << std::endl;

      }


      float w_000 = w[0][0] * w[1][0] * w[2][0];
      float w_001 = w[0][0] * w[1][0] * w[2][1];
      float w_010 = w[0][0] * w[1][1] * w[2][0];
      float w_011 = w[0][0] * w[1][1] * w[2][1];
      float w_100 = w[0][1] * w[1][0] * w[2][0];
      float w_101 = w[0][1] * w[1][0] * w[2][1];
      float w_110 = w[0][1] * w[1][1] * w[2][0];
      float w_111 = w[0][1] * w[1][1] * w[2][1];

      this->pin(idx[0][0], idx[1][0], idx[2][0], w_000 * weight);
      this->pin(idx[0][0], idx[1][0], idx[2][1], w_001 * weight);
      this->pin(idx[0][0], idx[1][1], idx[2][0], w_010 * weight);
      this->pin(idx[0][0], idx[1][1], idx[2][1], w_011 * weight);
      this->pin(idx[0][1], idx[1][0], idx[2][0], w_100 * weight);
      this->pin(idx[0][1], idx[1][0], idx[2][1], w_101 * weight);
      this->pin(idx[0][1], idx[1][1], idx[2][0], w_110 * weight);
      this->pin(idx[0][1], idx[1][1], idx[2][1], w_111 * weight);

      this->pin_counter++;
      delete[] values;
      delete[] centers;
      
      delete[] idx[0];
      delete[] idx[1];
      delete[] idx[2];
      delete[] idx;
      
      delete[] w[0];
      delete[] w[1];
      delete[] w[2];
      delete[] w;
    }

    void
    HistogramND::normalize ()
    {
      if (this->pin_counter > 0)
      {
        for (int i = 0; i < this->full_size; i++)
        {
          this->data[i] = this->data[i] / (float) this->pin_counter;
        }
      }
    }

    void
    HistogramND::insertInMat (cv::Mat& out, int row)
    {

      for (int j = 0; j < out.cols; j++)
      {
        if (j < this->n_bins)
        {
          if (isnan(this->data[j]))
          {
            out.at<float>(row, j) = 0;
          }
          else
          {
            out.at<float>(row, j) = this->data[j];
          }

        }
      }
    }




  }
}