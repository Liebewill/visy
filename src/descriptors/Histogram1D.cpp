/* 
 * File:   Histogram1D.cpp
 * Author: daniele
 * 
 * Created on 10 marzo 2015, 21.06
 */

#include "Histogram1D.h"
#include "tools.h"


namespace visy
{
  namespace descriptors
  {

    Histogram1D::Histogram1D (float range, int n_bins)
    {
      this->range = range;
      this->n_bins = n_bins;
      this->data = new float[n_bins];
      std::fill(this->data, this->data + n_bins, 0);
      this->pin_counter = 0;
    }

    Histogram1D::~Histogram1D ()
    {
      delete[] this->data;
    }

    void
    Histogram1D::pinValue (float value)
    {
      int bin_1, bin_2;
      float w1, w2;
      visy::tools::pinBin(this->range, this->n_bins, value, bin_1, bin_2, w1, w2);
      this->data[bin_1] += w1;
      this->data[bin_2] += w2;
      this->pin_counter++;
    }

    void
    Histogram1D::normalize ()
    {
      if (this->pin_counter > 0)
      {
        for (int i = 0; i < this->n_bins; i++)
        {
          this->data[i] = this->data[i] / (float) (this->pin_counter);
        }
      }
    }

    void
    Histogram1D::concat (Histogram1D& h)
    {
      float* temp = this->data;
      this->data = new float[this->n_bins + h.n_bins];
      std::copy(temp, temp + this->n_bins, this->data);
      std::copy(h.data, h.data + h.n_bins, this->data + this->n_bins);
      this->n_bins = this->n_bins + h.n_bins;
    }

    void
    Histogram1D::insertInMat (cv::Mat& out, int row)
    {

      for (int j = 0; j < out.cols; j++)
      {
        if (j < this->n_bins)
        {
          out.at<float>(row, j) = this->data[j];
        }
      }
    }

    std::ostream& operator<< (std::ostream& stream, const visy::descriptors::Histogram1D& h)
    {
      stream << "[";
      for (int i = 0; i < h.n_bins; i++)
      {
        stream << h.data[i];
        if (i < h.n_bins - 1)
        {
          stream << ";";
        }
      }
      stream << "]";

      return stream;
    }


  }
}

