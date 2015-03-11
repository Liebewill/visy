/* 
 * File:   Bold3DRDetector.cpp
 * Author: daniele
 * 
 * Created on 11 marzo 2015, 17.52
 */

#include "Bold3DRDetector.h"
#include "Bold3DExtractor.h"
#include "Bold3DDescriptorRadiusBunch.h"

namespace visy
{
  namespace detectors
  {

    Bold3DRDetector::Bold3DRDetector (std::vector<float>& radiuses, int n_bins, bool filter_occlusion, float zone_radius, float zone_slice, float area_normals_angular_th, float area_max_distance) : Detector ()
    {
      this->extractor = new visy::extractors::Bold3DExtractor(filter_occlusion, zone_radius, zone_slice, area_normals_angular_th, area_max_distance);
      this->descriptor = new visy::descriptors::Bold3DDescriptorRadiusBunch(n_bins, radiuses);
      this->multiplication_factor = radiuses.size();
    }

    void
    Bold3DRDetector::detect (cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask)
    {
      std::vector<visy::extractors::KeyPoint3D> temp_keypoints;
      this->extractor->extract(source, cloud, temp_keypoints, mask);
      this->descriptor->describe(source, cloud, temp_keypoints, descriptor);
      visy::extractors::utils::replicateKeypoints(temp_keypoints, keypoints, this->multiplication_factor);
    }

    Bold3DRDetector::~Bold3DRDetector ()
    {
    }

    std::string
    Bold3DRDetector::buildNameImpl ()
    {
      return "BOLD3DR";
    }

  }
}