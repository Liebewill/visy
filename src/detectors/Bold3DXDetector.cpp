/* 
 * File:   Bold3DXDetector.cpp
 * Author: daniele
 * 
 * Created on 20 marzo 2015, 22.39
 */

#include <opencv2/nonfree/features2d.hpp>

#include "Bold3DXDetector.h"
#include "Bold3DExtractor.h"
#include "Bold3DDescriptorMultiBunch.h"
#include "DFunctionB3DV2.h"
#include "SiftExtractor.h"
#include "DFunctionB3DV2Multi.h"

namespace visy
{
  namespace detectors
  {

    Bold3DXDetector::Bold3DXDetector (std::vector<float>& sizes, int n_bins, bool filter_occlusion, float zone_radius, float zone_slice, float area_normals_angular_th, float area_max_distance)
    {
      this->extractor = new visy::extractors::SiftExtractor(); // Bold3DExtractor(filter_occlusion, zone_radius, zone_slice, area_normals_angular_th, area_max_distance);
      this->descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(n_bins, sizes, new visy::descriptors::DFunctionB3DV2Multi(n_bins), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
      this->multiplication_factor = sizes.size();
      this->detector_embedded = true;
    }

    void
    Bold3DXDetector::detect (cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask)
    {
      std::vector<visy::extractors::KeyPoint3D> temp_keypoints;
      std::vector<visy::extractors::KeyPoint3D> temp_keypoints_2;
      this->extractor->extract(source, cloud, temp_keypoints, mask);

      visy::extractors::Bold3DExtractor b3dextractor;
      b3dextractor.extract(source, cloud, temp_keypoints_2, mask);

      temp_keypoints.insert(temp_keypoints.end(), temp_keypoints_2.begin(), temp_keypoints_2.end());

      this->descriptor->describe(source, cloud, temp_keypoints_2, descriptor);
      visy::extractors::utils::replicateKeypoints(temp_keypoints_2, keypoints, this->multiplication_factor);
    }

    void
    Bold3DXDetector::refineKeyPoints3D (std::vector<visy::extractors::KeyPoint3D>& keypoints_in, std::vector<visy::extractors::KeyPoint3D>& keypoints_out)
    {
      pcl::PointCloud<PointType>::Ptr keypoints_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType> ());
      pcl::KdTreeFLANN<PointType> kdtree;

      visy::extractors::utils::buildPrimiteCloudFromKeypoints(keypoints_cloud, keypoints_in);
      kdtree.setInputCloud(keypoints_cloud);

      std::vector<bool> kp_map(keypoints_in.size());
      std::fill(kp_map.begin(), kp_map.end(), true);

      std::vector<int> found_indices;
      std::vector<float> indices_distances;
      PointType searchPoint;
      for (int i = 0; i < keypoints_in.size(); i++)
      {
        found_indices.clear();
        indices_distances.clear();

        searchPoint.x = keypoints_in[i].pt3D.x;
        searchPoint.y = keypoints_in[i].pt3D.y;
        searchPoint.z = keypoints_in[i].pt3D.z;

        kdtree.radiusSearchT(searchPoint, 0.003f, found_indices, indices_distances);

        cv::Point3f p = keypoints_in[i].pt3D;
        cv::Point3f p_1 = keypoints_in[i].pt3D_1;
        cv::Point3f p_2 = keypoints_in[i].pt3D_2;
        for (int si = 1; si < found_indices.size(); si++)
        {
          kp_map[found_indices[si]] = false;
          p += keypoints_in[found_indices[si]].pt3D;
          p_1 += keypoints_in[found_indices[si]].pt3D_1;
          p_2 += keypoints_in[found_indices[si]].pt3D_2;
        }
        p = p * (1.0f / (float) found_indices.size());
        p_1 = p_1 * (1.0f / (float) found_indices.size());
        p_2 = p_2 * (1.0f / (float) found_indices.size());
        keypoints_in[i].pt3D = p;
        keypoints_in[i].pt3D_1 = p_1;
        keypoints_in[i].pt3D_2 = p_2;
      }

      for (int i = 0; i < kp_map.size(); i++)
      {
        if (kp_map[i] == true)
        {
          keypoints_out.push_back(keypoints_in[i]);
        }
      }

    }

    Bold3DXDetector::~Bold3DXDetector ()
    {
    }

    std::string
    Bold3DXDetector::buildNameImpl ()
    {
      return "BOLD3DRX";
    }
  }
}

