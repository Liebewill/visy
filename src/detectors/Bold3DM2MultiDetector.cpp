/* 
 * File:   Bold3DM2MultiDetector.cpp
 * Author: daniele
 * 
 * Created on 20 marzo 2015, 18.19
 */

#include "Bold3DM2MultiDetector.h"
#include "DFunctionB3DV2Multi.h"
#include "Bold3DExtractor.h"
#include "Bold3DDescriptorMultiBunch.h"

namespace visy
{
  namespace detectors
  {

    Bold3DM2MultiDetector::Bold3DM2MultiDetector (std::vector<float>& sizes, int n_bins, bool filter_occlusion, float zone_radius, float zone_slice, float area_normals_angular_th, float area_max_distance) : Detector ()
    {
      this->extractor = new visy::extractors::Bold3DExtractor(filter_occlusion, zone_radius, zone_slice, area_normals_angular_th, area_max_distance);
      this->descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(n_bins, sizes, new visy::descriptors::DFunctionB3DV2Multi(n_bins), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_KNN);
      this->multiplication_factor = sizes.size();
    }

    Bold3DM2MultiDetector::~Bold3DM2MultiDetector ()
    {
    }

    void
    Bold3DM2MultiDetector::detect (cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask)
    {
      std::vector<visy::extractors::KeyPoint3D> temp_keypoints;
      this->extractor->extract(source, cloud, temp_keypoints, mask);
      this->descriptor->describe(source, cloud, temp_keypoints, descriptor);
      visy::extractors::utils::replicateKeypoints(temp_keypoints, keypoints, this->multiplication_factor);
    }

    std::string
    Bold3DM2MultiDetector::buildNameImpl ()
    {
      return "BOLD3DM2MULTI";
    }

  }
}
