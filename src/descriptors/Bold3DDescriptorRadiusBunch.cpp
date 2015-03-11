/* 
 * File:   Bold3DDescriptorRadiusBunch.cpp
 * Author: daniele
 * 
 * Created on 11 marzo 2015, 16.21
 */

#include "Bold3DDescriptorRadiusBunch.h"

namespace visy
{
  namespace descriptors
  {

    Bold3DDescriptorRadiusBunch::Bold3DDescriptorRadiusBunch (int n_bins, std::vector<float>& radiuses) : Descriptor()
    {
      this->n_bins = n_bins;
      this->size = n_bins * 3;
      this->radiuses.insert(this->radiuses.end(), radiuses.begin(), radiuses.end());
    }

    Bold3DDescriptorRadiusBunch::~Bold3DDescriptorRadiusBunch ()
    {
    }

    void
    Bold3DDescriptorRadiusBunch::pairKeyPoint3D (visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results)
    {
      float* v = new float[3];

      cv::Point3f v1 = kp1.direction_x;
      cv::Point3f v2 = kp2.direction_x;

      cv::Point3f vm = kp2.pt3D - kp1.pt3D;


      cv::Point3f na = v1.cross(vm);
      cv::Point3f nb = v2.cross(vm);

      float alpha = v1.dot(vm);
      float beta = v2.dot(vm);
      float gamma = na.dot(nb);

      float alpha_c = acos(alpha / (cv::norm(v1) * cv::norm(vm)));
      float beta_c = acos(beta / (cv::norm(v2) * cv::norm(vm)));
      float gamma_c = acos(gamma / (cv::norm(na) * cv::norm(nb)));

      v[0] = alpha_c * (180 / M_PI);
      v[1] = beta_c * (180 / M_PI);
      v[2] = gamma_c * (180 / M_PI);

      *results = v;

    }

    void
    Bold3DDescriptorRadiusBunch::describe (cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor)
    {
      //DESCRIPTOR INIT
      int full_size = keypoints.size() * this->radiuses.size();
      descriptor = cv::Mat::zeros(full_size, this->size, CV_32F);


      //BUILDS KDTREE SPACE SEARCH
      pcl::PointCloud<PointType>::Ptr keypoints_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType> ());
      pcl::KdTreeFLANN<PointType> kdtree;

      visy::extractors::utils::buildPrimiteCloudFromKeypoints(keypoints_cloud, keypoints);
      kdtree.setInputCloud(keypoints_cloud);


      //SEARCHING IN BUNCHES
      std::vector<int> found_indices;
      std::vector<float> indices_distances;
      PointType searchPoint;


      for (int j = 0; j < this->radiuses.size(); j++)
      {
        float radius = this->radiuses[j];

        for (int i = 0; i < keypoints.size(); i++)
        {

          searchPoint.x = keypoints[i].pt3D.x;
          searchPoint.y = keypoints[i].pt3D.y;
          searchPoint.z = keypoints[i].pt3D.z;

          found_indices.clear();
          indices_distances.clear();
          kdtree.radiusSearch(searchPoint, radius, found_indices, indices_distances);


          Histogram1D h0(180.0f, this->n_bins);
          Histogram1D h1(180.0f, this->n_bins);
          Histogram1D h2(180.0f, this->n_bins);

          //START FROM 1 BECAUSE FIRST IS SEARCHPOINT ITSELF
          for (int index = 1; index < found_indices.size(); index++)
          {
            float* v;
            pairKeyPoint3D(keypoints[i], keypoints[found_indices[index]], &v);

            h0.pinValue(v[0]);
            h1.pinValue(v[1]);
            h2.pinValue(v[2]);
          }

          h0.normalize();
          h1.normalize();
          h2.normalize();

          h0.concat(h1);
          h0.concat(h2);

          h0.insertInMat(descriptor, i + (j * keypoints.size()));
        }

      }
    }

    std::string
    Bold3DDescriptorRadiusBunch::buildNameImpl ()
    {
      std::stringstream ss;
      ss << "BOLD3D-RADIUSBUNCH;";
      ss << this->n_bins << ";";

      ss << "(";
      for (int i = 0; i < this->radiuses.size(); i++)
      {
        ss << this->radiuses[i];
        if (i < this->radiuses.size() - 1)
        {
          ss << ",";
        }
      }
      ss << ")";
      return ss.str();
    }


  }
}