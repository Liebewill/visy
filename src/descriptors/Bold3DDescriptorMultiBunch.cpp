/* 
 * File:   Bold3DDescriptor.cpp
 * Author: daniele
 * 
 * Created on 10 marzo 2015, 18.47
 */

#include <vector>

#include "Bold3DDescriptorMultiBunch.h"


namespace visy
{
  namespace descriptors
  {

    Bold3DDescriptorMultiBunch::Bold3DDescriptorMultiBunch (int n_bins, std::vector<float>& sizes, int bunch_method) : Descriptor ()
    {
      this->n_bins = n_bins;
      this->size = n_bins * 3;
      this->sizes.insert(this->sizes.end(), sizes.begin(), sizes.end());
      this->bunch_method = bunch_method;
    }

    Bold3DDescriptorMultiBunch::~Bold3DDescriptorMultiBunch ()
    {
    }

    void
    Bold3DDescriptorMultiBunch::pairKeyPoint3D (visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results)
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


      //SEARCHING IN BUNCHES
      std::vector<int> found_indices;
      std::vector<float> indices_distances;
      PointType searchPoint;


      for (int j = 0; j < this->sizes.size(); j++)
      {
        float s = this->sizes[j];

        if (bunch_method == BUNCH_METHOD_KNN)
        {
          s += 1.0f;
        }

        for (int i = 0; i < keypoints.size(); i++)
        {

          searchPoint.x = keypoints[i].pt3D.x;
          searchPoint.y = keypoints[i].pt3D.y;
          searchPoint.z = keypoints[i].pt3D.z;

          found_indices.clear();
          indices_distances.clear();
          if (this->bunch_method == BUNCH_METHOD_KNN)
          {
            kdtree.nearestKSearch(searchPoint, s, found_indices, indices_distances);
          }
          else if (this->bunch_method == BUNCH_METHOD_RADIUS)
          {
            kdtree.radiusSearchT(searchPoint, s, found_indices, indices_distances);
          }


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