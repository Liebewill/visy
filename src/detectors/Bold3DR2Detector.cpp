/* 
 * File:   Bold3DR2Detector.cpp
 * Author: daniele
 * 
 * Created on 11 marzo 2015, 17.52
 */

#include "Bold3DR2Detector.h"
#include "Bold3DExtractor.h"
#include "Bold3DDescriptorMultiBunch.h"
#include "DFunctionB3DV2.h"

namespace visy
{
  namespace detectors
  {

    Bold3DR2Detector::Bold3DR2Detector (std::vector<float>& radiuses, int n_bins, bool filter_occlusion, float zone_radius, float zone_slice, float area_normals_angular_th, float area_max_distance) : Detector ()
    {
      this->extractor = new visy::extractors::Bold3DExtractor(filter_occlusion, zone_radius, zone_slice, area_normals_angular_th, area_max_distance);
      this->descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(n_bins, radiuses,new visy::descriptors::DFunctionB3DV2(n_bins), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
      this->multiplication_factor = radiuses.size();
    }

    void
    Bold3DR2Detector::detect (cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask)
    {
      std::vector<visy::extractors::KeyPoint3D> temp_keypoints;
      this->extractor->extract(source, cloud, temp_keypoints, mask);
      this->descriptor->describe(source, cloud, temp_keypoints, descriptor);
      visy::extractors::utils::replicateKeypoints(temp_keypoints, keypoints, this->multiplication_factor);
    }

    

    Bold3DR2Detector::~Bold3DR2Detector ()
    {
    }

    std::string
    Bold3DR2Detector::buildNameImpl ()
    {
      return "BOLD3DR2";
    }

  }
}