/* 
 * File:   Bold3DDescriptor.cpp
 * Author: daniele
 * 
 * Created on 10 marzo 2015, 18.47
 */

#include <vector>

#include "Bold3DDescriptorMultiBunch.h"
#include "descriptors_utils.h"


namespace visy
{
  namespace descriptors
  {

    Bold3DDescriptorMultiBunch::Bold3DDescriptorMultiBunch (int n_bins, std::vector<float>& sizes, DFunction* dfunction, int bunch_method) : Descriptor (dfunction)
    {
      this->n_bins = n_bins;
      this->size = dfunction->getDataSize();
      this->sizes.insert(this->sizes.end(), sizes.begin(), sizes.end());
      this->bunch_method = bunch_method;
     
    }

    Bold3DDescriptorMultiBunch::~Bold3DDescriptorMultiBunch ()
    {
    }

    void
    Bold3DDescriptorMultiBunch::describe (cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor)
    {
      //DESCRIPTOR INIT
      int full_size = keypoints.size() * this->sizes.size();
      descriptor = cv::Mat::zeros(full_size, this->size, CV_32F);


      //BUILDS KDTREE SPACE SEARCH
      pcl::PointCloud<PointType>::Ptr keypoints_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType> ());
      pcl::KdTreeFLANN<PointType> kdtree;

      visy::extractors::utils::buildPrimiteCloudFromKeypoints(keypoints_cloud, keypoints);
      kdtree.setInputCloud(keypoints_cloud);

      for (int i = 0; i < keypoints.size(); i++)
      {
        visy::descriptors::utils::multiBunchDescription(keypoints[i],keypoints,kdtree, sizes,bunch_method,dfunction,descriptor,i);
      }
    }

    std::string
    Bold3DDescriptorMultiBunch::buildNameImpl ()
    {
      std::stringstream ss;
      ss << "BOLD3D-MULTIBUNCH;";
      ss << this->n_bins << ";";
      ss << this->bunch_method << ";";

      ss << "(";


      for (int i = 0; i < this->sizes.size(); i++)
      {
        ss << this->sizes[i];
        if (i < this->sizes.size() - 1)
        {
          ss << ",";
        }
      }
      ss << ")";
      return ss.str();
    }


  }
}