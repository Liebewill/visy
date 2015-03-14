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

    Bold3DMDetector::Bold3DMDetector (std::vector<float>& sizes, int n_bins, bool filter_occlusion, float zone_radius, float zone_slice, float area_normals_angular_th, float area_max_distance) : Detector ()
    {
      this->extractor = new visy::extractors::Bold3DExtractor(filter_occlusion, zone_radius, zone_slice, area_normals_angular_th, area_max_distance);
      this->descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(n_bins, sizes, visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_KNN);
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

//    void
//    Bold3DMDetector::refineKeyPoints3D (std::vector<visy::extractors::KeyPoint3D>& keypoints_in, cv::Mat& descriptor_in, std::vector<visy::extractors::KeyPoint3D>& keypoints_out, cv::Mat& descriptor_out)
//    {
//      pcl::PointCloud<PointType>::Ptr keypoints_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType> ());
//      pcl::KdTreeFLANN<PointType> kdtree;
//
//      visy::extractors::utils::buildPrimiteCloudFromKeypoints(keypoints_cloud, keypoints_in);
//      kdtree.setInputCloud(keypoints_cloud);
//      
//      std::vector<bool> excluding_mask(keypoints_in.size());
//      std::fill(excluding_mask.begin(),excluding_mask.end(),true);
//      
//      for (int i = 0; i < keypoints_in.size(); i++)
//      {
//        if(excluding_mask[i]==false)continue;
//        
//        std::vector<int> found_indices;
//        std::vector<float> indices_distances;
//        PointType searchPoint;
//
//        searchPoint.x = keypoints_in[i].pt3D.x;
//        searchPoint.y = keypoints_in[i].pt3D.y;
//        searchPoint.z = keypoints_in[i].pt3D.z;
//        
//        kdtree.radiusSearch(searchPoint,0.01f,found_indices,indices_distances);
//        
//        for(int f = 0; f < found_indices.size(); f++){
//          if(keypoints_in[found_indices[f]].pt3D != keypoints_in[i].pt3D){
//            excluding_mask[found_indices[f]]=false;
//          }
//        }
//        
//      }
//      
//       int w = descriptor_in.cols;
//      for(int i =0; i < keypoints_in.size(); i++){
//        if(excluding_mask[i]){
//          if(keypoints_out.size()==0){
//            descriptor_out = cv::Mat::zeros(1,w,CV_32F);
//             descriptor_out(cv::Rect_<int>(0, 0, w, 1)) = descriptor_in(cv::Rect_<int>(0, i, w, 1));
//          }else{
//            cv::vconcat(descriptor_out,descriptor_in(cv::Rect_<int>(0, i, w, 1)),descriptor_out);
//          }
//          keypoints_out.push_back(keypoints_in[i]);
//        }
//      }
//
//
//    }

    std::string
    Bold3DMDetector::buildNameImpl ()
    {
      return "BOLD3DM";
    }


  }
}