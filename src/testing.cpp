/*
 * File:   bunch_tester.cpp
 * Author: daniele
 *
 * Created on 27 novembre 2014, 22.17
 */

#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boldlib.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/intersections.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/centroid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>


#include <sstream>
#include <string>
#include <fstream>

#include <tools.h>
#include <opencv2/core/mat.hpp>

#include "Extractor.h"
#include "Bold3DExtractor.h"
#include "IrosDataset.h"
#include "extractors/extrators_utils.h"
#include "Descriptor.h"
#include "Bold3DDescriptorMultiBunch.h"
#include "Detector.h"
#include "Bold3DMDetector.h"
#include "Bold3DRDetector.h"
#include "BoldDetector.h"

using namespace std;
using namespace BoldLib;


/*
LineSegmentDetection( int * n_out,
double * img, int X, int Y,
double scale, double sigma_scale, double quant,
double ang_th, double log_eps, double density_th,
int n_bins,
int ** reg_img, int * reg_x, int * reg_y );
 */

/*
 *
 */
int
main (int argc, char** argv)
{

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  for (int sce = 10; sce < 15; sce++)
  {
    cv::Mat model_rgb, model_rgb_full;
    pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr model_cloud_full(new pcl::PointCloud<PointType>());
    Eigen::Matrix4f model_pose;

    //LOAD MODEL
    visy::dataset::IrosDataset::loadModel(
            "asus_box", 1,
            model_cloud_full, model_cloud,
            model_rgb, model_rgb_full,
            model_pose);



    cv::Mat scene_rgb, scene_rgb_full;
    pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr scene_cloud_full(new pcl::PointCloud<PointType>());
    Eigen::Matrix4f scene_pose;

    visy::dataset::IrosDataset::loadScene(14,sce, scene_cloud, scene_rgb);

    //  visy::dataset::IrosDataset::loadModel(
    //          "asus_box", 2,
    //          scene_cloud, scene_cloud_full,
    //          scene_rgb_full, scene_rgb,
    //          scene_pose);


    std::vector<float> sizes;
    sizes.push_back(5);
    sizes.push_back(10);
    sizes.push_back(15);
    sizes.push_back(20);
    sizes.push_back(25);
    sizes.push_back(30);

    std::vector<float> radiuses;
    radiuses.push_back(0.001f);
    radiuses.push_back(0.01f);
    radiuses.push_back(0.1f);


    std::vector<visy::extractors::KeyPoint3D> model_keypoints;
    std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
    cv::Mat model_descriptor;
    cv::Mat scene_descriptor;

//      visy::detectors::Detector * detector = new visy::detectors::Bold3DMDetector(sizes,8,true);
    visy::detectors::Detector * detector = new visy::detectors::Bold3DRDetector(radiuses,8);
//    visy::detectors::Detector * detector = new visy::detectors::BoldDetector(sizes);

    std::cout << "USING DETECTOR: " << detector->buildName() << std::endl;

    detector->detect(model_rgb, model_cloud_full, model_keypoints, model_descriptor);
    detector->detect(scene_rgb, scene_cloud, scene_keypoints, scene_descriptor);

    //
    //  visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(true);
    //
    //  std::cout << "EXT: " << extractor->buildName() << std::endl;
    //
    //  
    //  extractor->extract(model_rgb, model_cloud_full, model_keypoints);
    //  extractor->extract(scene_rgb, scene_cloud, scene_keypoints);
    //
    //
    std::cout << "MODEL KPS:" << model_keypoints.size() << std::endl;
    std::cout << "SCENE KPS:" << scene_keypoints.size() << std::endl;
    //
    //  
    //
    //
    //
    ////    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(12, sizes);
    //  visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorRadiusBunch(12, radiuses);
    //
    //  std::cout << "DESCR: " << descriptor->buildName() << std::endl;
    //
    //  descriptor->describe(model_rgb, model_cloud_full, model_keypoints, model_descriptor);
    //  descriptor->describe(scene_rgb, scene_cloud, scene_keypoints, scene_descriptor);
    //
    //  std::vector<visy::extractors::KeyPoint3D> model_keypoints_p;
    //  std::vector<visy::extractors::KeyPoint3D> scene_keypoints_p;
    //
    //  for (int i = 0; i < sizes.size(); i++)
    //  {
    //    model_keypoints_p.insert(model_keypoints_p.end(), model_keypoints.begin(), model_keypoints.end());
    //    scene_keypoints_p.insert(scene_keypoints_p.end(), scene_keypoints.begin(), scene_keypoints.end());
    //  }
    //  model_keypoints = model_keypoints_p;
    //  scene_keypoints = scene_keypoints_p;

    /*MATCHES*/
    /* Keypoints Matching */
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;
    visy::tools::matchKeypointsBrute(matches, good_matches, model_descriptor, scene_descriptor);

    std::cout << "MAtch found:" << good_matches.size() << std::endl;

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

    std::cout << "Model Filtered:" << matched_model_keypoints.size() << std::endl;
    std::cout << "Scene Filtered:" << matched_scene_keypoints.size() << std::endl;
    std::cout << "Model Scene corss:" << model_scene_corrs->size() << std::endl;

    /* GEOMETRU CONSISTENCY GROUPING*/
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector < pcl::Correspondences > clustered_corrs;
    visy::extractors::utils::keypointsGeometricConsistencyGrouping(0.01f, 10,
            matched_model_keypoints,
            matched_scene_keypoints,
            model_scene_corrs,
            rototranslations,
            clustered_corrs);



    std::cout << "FOUND:" << rototranslations.size() << std::endl;



    cv::Mat out = model_rgb_full.clone();
    cv::Mat out_scene = scene_rgb.clone();
    visy::extractors::utils::draw3DKeyPointsWithAreas(out, model_keypoints, cv::Scalar(255, 0, 0), 1.0f, 5.0f, 2.0f);
    visy::extractors::utils::draw3DKeyPointsWithAreas(out_scene, scene_keypoints, cv::Scalar(255, 0, 0), 1.0f, 5.0f, 2.0f);


    pcl::visualization::PCLVisualizer * viewer;
    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");


    viewer->addPointCloud(scene_cloud, "scene");
    //  visy::extractors::KeyPoint3D::draw3DKeyPoints3D(*viewer, matched_scene_keypoints, cv::Scalar(0, 255, 0), "ciao");

    for (int i = 0; i < rototranslations.size(); i++)
    {
      pcl::PointCloud<PointType>::Ptr model_projection(new pcl::PointCloud<PointType>());
      pcl::transformPointCloud(*model_cloud, *model_projection, rototranslations[i]);
      std::stringstream ss;
      ss << "Instance_" << i << "_cloud";
      visy::tools::displayCloud(*viewer, model_projection, 0, 255, 0, 3.0f, ss.str());
    }

    cv::namedWindow("out", cv::WINDOW_NORMAL);
    cv::imshow("out", out);
    cv::namedWindow("out_scene", cv::WINDOW_NORMAL);
    cv::imshow("out_scene", out_scene);

    while (!viewer->wasStopped())
    {
      cv::waitKey(100);
      viewer->spinOnce();
    }



    std::cout << "EDGE DONE" << std::endl;
  }


  return 1;
}
