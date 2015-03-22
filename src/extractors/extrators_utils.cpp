/* 
 * File:   Utils.cpp
 * Author: daniele
 * 
 * Created on 9 marzo 2015, 21.43
 */

#include "extrators_utils.h"
#include "Bold3DExtractor.h"

namespace visy
{
  namespace extractors
  {
    namespace utils
    {

      /**
       * 
       * @param kp
       * @param source_size
       * @param area
       * @param radius
       * @param slice
       */
      void
      extractSliceAreaFromKeypoint3D (KeyPoint3D& kp, cv::Size2i source_size, std::vector<int>& area, float radius, float slice)
      {
        int max = source_size.width * source_size.height - 1;
        area.clear();
        int index = -1;
        int startx = kp.pt.x - kp.size;
        int starty = kp.pt.y - kp.size;
        startx = startx < 0 ? 0 : startx;
        starty = starty < 0 ? 0 : starty;
        for (int y = starty; y < starty + kp.size * 2; y++)
        {
          for (int x = startx; x < startx + kp.size * 2; x++)
          {
            cv::Point2f tp(x, y);
            double dis = cv::norm(kp.pt - tp);
            if (dis <= radius)
            {
              double slice_dis = visy::tools::point2LineDistance(kp.pt1, kp.pt2, tp);
              if (slice_dis > slice)
              {
                index = tp.x + tp.y * source_size.width;
                if (index > max)index = max;
                if (index < 0) index = 0;
                area.push_back(index);
              }
            }
          }
        }
      }

      /**
       * 
       * @param kp
       * @param source_size
       * @param area_left
       * @param area_right
       * @param radius
       * @param slice
       */
      void
      extractSliceAreaPairFromKeypoint3D (KeyPoint3D& kp, cv::Size2i source_size, std::vector<int>& area_left, std::vector<int>& area_right, float radius, float slice)
      {
        std::vector<int> area;
        extractSliceAreaFromKeypoint3D(kp, source_size, area, radius, slice);

        area_left.clear();
        area_right.clear();

        for (int i = 0; i < area.size(); i++)
        {
          int index = area[i];
          cv::Point2i tp(index % source_size.width, index / source_size.width);
          cv::Point2i tp_1 = kp.pt1 - tp;
          cv::Point2i tp_2 = kp.pt2 - tp;
          int dot = tp_1.cross(tp_2);
          if (dot >= 0)
          {
            area_right.push_back(index);
          }
          else
          {
            area_left.push_back(index);
          }
        }
      }

      /**
       * 
       * @param out
       * @param keypoints
       * @param color
       * @param tick
       * @param radius
       * @param slice
       */
      void
      draw3DKeyPointsWithAreas (cv::Mat& out, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Scalar color, float tick, float radius, float slice)
      {
        KeyPoint3D::draw3DKeyPoints(out, keypoints, color, tick);
        std::vector<int> area_left, area_right;
        for (int i = 0; i < keypoints.size(); i++)
        {
          area_left.clear();
          area_right.clear();
          visy::extractors::KeyPoint3D kp = keypoints[i];
          extractSliceAreaPairFromKeypoint3D(kp, cv::Size2i(out.cols, out.rows), area_left, area_right, radius, slice);

          for (int area_index = 0; area_index < area_left.size(); area_index++)
          {
            cv::circle(out, cv::Point(area_left[area_index] % out.cols, area_left[area_index] / out.cols), 1.0f, cv::Scalar(0, 0, 255));
          }

          for (int area_index = 0; area_index < area_right.size(); area_index++)
          {
            cv::circle(out, cv::Point(area_right[area_index] % out.cols, area_right[area_index] / out.cols), 1.0f, cv::Scalar(0, 255, 0));
          }
        }

      }

      /**
       * 
       * @param cloud
       * @param keypoints
       */
      void
      buildPrimiteCloudFromKeypoints (pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints)
      {
        for (int i = 0; i < keypoints.size(); i++)
        {
          PointType pt;

          pt.x = keypoints[i].pt3D.x;
          pt.y = keypoints[i].pt3D.y;
          pt.z = keypoints[i].pt3D.z;

          cloud->points.push_back(pt);
        }
      }

      /**
       * 
       * @param model_keypoints
       * @param scene_keypoints
       * @param matches
       * @param matched_model_keypoints
       * @param matched_scene_keypoints
       * @param matched_model_keypoints_indices
       * @param matched_scene_keypoints_indices
       * @param model_scene_corrs
       */
      void
      keypointsConsensusSet (
              std::vector<visy::extractors::KeyPoint3D >& model_keypoints,
              std::vector<visy::extractors::KeyPoint3D >& scene_keypoints,
              std::vector<cv::DMatch>& matches,
              std::vector<visy::extractors::KeyPoint3D>& matched_model_keypoints,
              std::vector<visy::extractors::KeyPoint3D>& matched_scene_keypoints,
              std::vector<int>& matched_model_keypoints_indices,
              std::vector<int>& matched_scene_keypoints_indices,
              pcl::CorrespondencesPtr& model_scene_corrs)
      {
        /* Good Keyline filtering */
        matched_model_keypoints.clear();
        matched_scene_keypoints.clear();
        matched_model_keypoints_indices.clear();
        matched_scene_keypoints_indices.clear();
        model_scene_corrs = pcl::CorrespondencesPtr(new pcl::Correspondences());

        for (int i = 0; i < matches.size(); i++)
        {
          matched_model_keypoints.push_back(model_keypoints[matches[i].trainIdx]);
          matched_scene_keypoints.push_back(scene_keypoints[matches[i].queryIdx]);
          matched_model_keypoints_indices.push_back(matches[i].trainIdx);
          matched_scene_keypoints_indices.push_back(matches[i].queryIdx);

          //        pcl::Correspondence corr(good_matches[i].trainIdx,good_matches[i].queryIdx,good_matches[i].distance);
          pcl::Correspondence corr(i, i, matches[i].distance);
          model_scene_corrs->push_back(corr);
        }

      }

      /**
       * 
       * @param gc_size
       * @param gc_th
       * @param model_keypoints
       * @param scene_keypoints
       * @param model_scene_corrs
       * @param rototranslations
       * @param clustered_corrs
       */
      void
      keypointsGeometricConsistencyGrouping (double gc_size, int gc_th,
              std::vector<visy::extractors::KeyPoint3D>& model_keypoints,
              std::vector<visy::extractors::KeyPoint3D>& scene_keypoints,
              pcl::CorrespondencesPtr& model_scene_corrs,
              std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& rototranslations,
              std::vector < pcl::Correspondences >& clustered_corrs)
      {
        pcl::PointCloud<PointType>::Ptr model_keypoints_cloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr scene_keypoints_cloud(new pcl::PointCloud<PointType>());
        buildPrimiteCloudFromKeypoints(model_keypoints_cloud, model_keypoints);
        buildPrimiteCloudFromKeypoints(scene_keypoints_cloud, scene_keypoints);


        pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
        gc_clusterer.setGCSize(gc_size);
        gc_clusterer.setGCThreshold(gc_th);

        gc_clusterer.setInputCloud(model_keypoints_cloud);
        gc_clusterer.setSceneCloud(scene_keypoints_cloud);
        gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

        rototranslations.clear();
        clustered_corrs.clear();

        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > temp_rototranslations;
        gc_clusterer.recognize(temp_rototranslations);
        for (int i = 0; i < temp_rototranslations.size(); i++)
        {
          if (temp_rototranslations[i].block<3, 1>(0, 3).norm() > 0.000001f)
          {
            rototranslations.push_back(temp_rototranslations[i]);
          }
        }

      }

      /**
       * 
       * @param source
       * @param out
       * @param times
       */
      void
      replicateKeypoints (std::vector<KeyPoint3D>& source, std::vector<KeyPoint3D>& out, int times)
      {
        for (int i = 0; i < times; i++)
        {
          out.insert(out.end(), source.begin(), source.end());
        }
      }

      /**
       * 
       * @param model_keypoints
       * @param scene_keypoints
       * @param model_descriptor
       * @param scene_descriptor
       * @param rototranslations
       */
      void
      modelSceneMatch (
              std::vector<visy::extractors::KeyPoint3D>& model_keypoints,
              std::vector<visy::extractors::KeyPoint3D>& scene_keypoints,
              cv::Mat& model_descriptor,
              cv::Mat& scene_descriptor,
              std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& rototranslations,
              float gc_size,
              float gc_th,
              int match_type)
      {
        /* MATCHES */
        std::vector<cv::DMatch> matches;
        std::vector<cv::DMatch> good_matches;
        visy::tools::matchKeypointsBrute(matches, good_matches, model_descriptor, scene_descriptor, match_type);

        /* KEYLINES CONSENSUS SET*/
        std::vector<visy::extractors::KeyPoint3D> matched_model_keypoints;
        std::vector<visy::extractors::KeyPoint3D> matched_scene_keypoints;
        std::vector<int> matched_model_keypoints_indices;
        std::vector<int> matched_scene_keypoints_indices;
        pcl::CorrespondencesPtr model_scene_corrs;
        visy::extractors::utils::keypointsConsensusSet(
                model_keypoints,
                scene_keypoints,
                good_matches,
                matched_model_keypoints,
                matched_scene_keypoints,
                matched_model_keypoints_indices,
                matched_scene_keypoints_indices,
                model_scene_corrs);

        /* GEOMETRU CONSISTENCY GROUPING*/
        std::vector < pcl::Correspondences > clustered_corrs;
        visy::extractors::utils::keypointsGeometricConsistencyGrouping(gc_size, gc_th,
                matched_model_keypoints,
                matched_scene_keypoints,
                model_scene_corrs,
                rototranslations,
                clustered_corrs);

      }

      /**
       * 
       * @param source
       * @param target
       */
      bool
      isInOcclusionSide (visy::extractors::KeyPoint3D& source, visy::extractors::KeyPoint3D& target)
      {
        if (
                isnan(target.pt3D.x) ||
                isnan(target.pt3D.y) ||
                isnan(target.pt3D.z) ||
                isnan(source.pt3D.x) ||
                isnan(source.pt3D.y) ||
                isnan(source.pt3D.z)
                )
          return false;

        cv::Point3f d = target.pt3D - source.pt3D;
        return target.direction_y.dot(d) < 0;
      }
      //END
    }
  }
}