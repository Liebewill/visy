#include "descriptors_utils.h"
#include "Bold3DDescriptorMultiBunch.h"
#include "signatures/Signature.h"

namespace visy
{
  namespace descriptors
  {
    namespace utils
    {

      void
      multiBunchDescription (visy::extractors::KeyPoint3D& kp3d, std::vector<visy::extractors::KeyPoint3D>& keypoints, std::vector<float>& sizes, int bunch_method, DFunction* dfunction, std::vector<Signature>& signatures)
      {
        //BUILDS KDTREE SPACE SEARCH
        pcl::PointCloud<PointType>::Ptr keypoints_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType> ());
        pcl::KdTreeFLANN<PointType> kdtree;

        visy::extractors::utils::buildPrimiteCloudFromKeypoints(keypoints_cloud, keypoints);
        kdtree.setInputCloud(keypoints_cloud);

        std::vector<int> found_indices;
        std::vector<float> indices_distances;
        PointType searchPoint;

        Signature* signature = dfunction->getSignature();
        
        for (int si = 0; si < sizes.size(); si++)
        {
          float size = sizes[si];
          if (bunch_method == Bold3DDescriptorMultiBunch::BUNCH_METHOD_KNN)
          {
            size += 1.0f;
          }

          searchPoint.x = kp3d.pt3D.x;
          searchPoint.y = kp3d.pt3D.y;
          searchPoint.z = kp3d.pt3D.z;

          found_indices.clear();
          indices_distances.clear();
          if (bunch_method == Bold3DDescriptorMultiBunch::BUNCH_METHOD_KNN)
          {
            kdtree.nearestKSearch(searchPoint, size, found_indices, indices_distances);
          }
          else if (bunch_method == Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS)
          {
            kdtree.radiusSearchT(searchPoint, size, found_indices, indices_distances);
          }

          signature->initialize();
          for (int index = 1; index < found_indices.size(); index++)
          {
            float* v;
            dfunction->f(kp3d, keypoints[found_indices[index]], &v);
            signature->pinValue(v);
            delete v;
          }

          signature->finalize();

          signatures.push_back(*signature);
        }
      }
    }
  }
}
