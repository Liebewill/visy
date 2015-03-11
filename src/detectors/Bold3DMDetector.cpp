/* 
 * File:   Bold3DMDetector.cpp
 * Author: daniele
 * 
 * Created on 11 marzo 2015, 17.30
 */

#include "Bold3DMDetector.h"
#include "Bold3DExtractor.h"
#include "Bold3DDescriptorMultiBunch.h"
namespace visy
{
  namespace detectors
  {

    Bold3DMDetector::Bold3DMDetector (std::vector<int>& sizes, int n_bins, bool filter_occlusion, float zone_radius, float zone_slice, float area_normals_angular_th, float area_max_distance) : Detector ()
    {
      this->extractor = new visy::extractors::Bold3DExtractor(filter_occlusion, zone_radius, zone_slice, area_normals_angular_th, area_max_distance);
      this->descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(n_bins, sizes);
      this->multiplication_factor = sizes.size();
    }

    Bold3DMDetector::~Bold3DMDetector ()
    {
    }

    void
    Bold3DMDetector::detect (cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask)
    { 
      std::vector<visy::extractors::KeyPoint3D> temp_keypoints;
      this->extractor->extract(source, cloud, temp_keypoints, mask);
      this->descriptor->describe(source, cloud, temp_keypoints, descriptor);
      visy::extractors::utils::replicateKeypoints(temp_keypoints, keypoints, this->multiplication_factor);
    }

    std::string
    Bold3DMDetector::buildNameImpl ()
    {
      return "BOLD3DM" ;
    }


  }
}