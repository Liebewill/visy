/* 
 * File:   Signature.cpp
 * Author: daniele
 * 
 * Created on 15 marzo 2015, 14.35
 */

#include "signatures/Signature.h"

namespace visy
{
  namespace descriptors
  {
    Signature::Signature (int dimension, float range, int n_bins, bool sparse)
    {
      this->dimension = dimension;
      this->range = range;
      this->n_bins = n_bins;
      this->sparse = false;
      this->initialize();
    }

    void
    Signature::initialize ()
    {

      if (!this->sparse)
      {
        this->histograms.clear();
        for (int i = 0; i < dimension; i++)
        {
          this->histograms.push_back(boost::shared_ptr<Histogram1D>(new Histogram1D(range, n_bins)));
        }
      }
    }

    int
    Signature::getSize ()
    {
      return this->n_bins * this->dimension;
    }

    Signature::~Signature ()
    {
      this->histograms.clear();
    }

    void
    Signature::pinValue (float* values)
    {
      for (int i = 0; i < dimension; i++)
      {
        this->histograms[i]->pinValue(values[i]);
      }
    }

    void
    Signature::finalize ()
    {
      for (int i = 0; i < dimension; i++)
      {
        this->histograms[i]->normalize();
      }

      for (int i = 1; i < dimension; i++)
      {
        this->histograms[0]->concat(*(this->histograms[i]));
      }
    }

    float*
    Signature::getData ()
    {
      return this->histograms[0]->data;
    }

    void
    Signature::insertInMat (cv::Mat& out, int row)
    {
      this->histograms[0]->insertInMat(out, row);
    }




  }
}