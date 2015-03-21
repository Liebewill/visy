/* 
 * File:   Signature.cpp
 * Author: daniele
 * 
 * Created on 15 marzo 2015, 14.35
 */

#include "signatures/Signature.h"
#include "signatures/HistogramND.h"

namespace visy
{
  namespace descriptors
  {
    int Signature::STATIC_COUNTER = 0;
    
    Signature::Signature (int dimension, float range, int n_bins, bool sparse)
    {
      this->dimension = dimension;
      this->range = range;
      this->n_bins = n_bins;
      this->sparse = sparse;
      this->multiHistogram = boost::shared_ptr<HistogramND>(new HistogramND(this->dimension, this->range, this->n_bins));
      this->initialize();
    }

    void
    Signature::initialize ()
    {
      this->histograms.clear();
      
      if (!this->sparse)
      {
        for (int i = 0; i < dimension; i++)
        {
          this->histograms.push_back(boost::shared_ptr<Histogram1D>(new Histogram1D(range, n_bins)));
        }
      }
      else
      {
        this->multiHistogram->reset();
      }
    }

    int
    Signature::getSize ()
    {
      if (!sparse)
      {
        return this->n_bins * this->dimension;
      }
      else
      {
        return pow(this->n_bins, this->dimension);
      }
    }

    Signature::~Signature ()
    {
      this->histograms.clear();
      this->multiHistogram.reset();

    }

    void
    Signature::pinValue (float* values)
    {
      if (!sparse)
      {
        for (int i = 0; i < dimension; i++)
        {
          this->histograms[i]->pinValue(values[i]);
        }
      }
      else
      {
        this->multiHistogram->placeVector(values);
      }
    }

    void
    Signature::finalize ()
    {
      if (!sparse)
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
      else
      {
        this->multiHistogram->normalize();
      }
    }

    float*
    Signature::getData ()
    {
      if (!sparse)
      {
        return this->histograms[0]->data;
      }
      else
      {
        return this->multiHistogram->data;
      }
    }

    void
    Signature::insertInMat (cv::Mat& out, int row)
    {
      if (!sparse)
      {
        this->histograms[0]->insertInMat(out, row);
      }
      else
      {
        this->multiHistogram->insertInMat(out, row);
      }
    }




  }
}