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

#include <opencv2/opencv.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include "detectors/detectors_utils.h"
#include "Parameters.h"
#include "WillowDataset.h"

using namespace std;
using namespace BoldLib;


visy::Parameters* parameters;
pcl::visualization::PCLVisualizer * viewer;

/*
 *
 */
int
main(int argc, char** argv) {

    /** PARAMETERS */
    parameters = new visy::Parameters(argc, argv);
    parameters->putFloat("gc_th");
    parameters->putFloat("gc_size");
    parameters->putString("detector");
    parameters->putString("model");
    parameters->putString("sizes");
    parameters->putInt("set");
    parameters->putInt("scene");
    parameters->putInt("nbin");
    parameters->putInt("occlusion");

    visy::detectors::Detector * detector;
    detector = visy::detectors::utils::buildDetectorFromString(parameters->getString("detector"), parameters, false);


    visy::dataset::WillowDataset dataset;

    dataset.init();

    for (int i = 0; i < dataset.scenes->size(); i++) {
        visy::dataset::SetScene set = dataset.scenes->at(i);
        
        for (int j= 0; j <= set.scene_number; j++) {


            std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
            cv::Mat scene_descriptor;
            cv::Mat scene_rgb, scene_rgb_full;
            pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());

            dataset.loadScene(set.set_number, j, scene_cloud, scene_rgb);

//            viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");
//            viewer->addPointCloud(scene_cloud, "cloud");
//
//            while (!viewer->wasStopped()) {
//                viewer->spinOnce();
//            }
        }
    }

    return 0;

    //    for (int i = 0; i < dataset.models->size(); i++) {
    //        visy::dataset::Model model = dataset.models->at(i);
    //
    //
    //        std::vector<visy::extractors::KeyPoint3D> model_keypoints;
    //        cv::Mat model_descriptor;
    //        pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
    //        pcl::PointCloud<PointType>::Ptr model_cloud_filtered(new pcl::PointCloud<PointType>());
    //        pcl::PointCloud<PointType>::Ptr model_full_cloud(new pcl::PointCloud<PointType>());
    //        cv::Mat model_rgb, model_rgb_full;
    //        Eigen::Matrix4f model_pose;
    //
    //
    //        std::cout << model.name << " " << model.n_views << std::endl;
    //
    //        dataset.fetchFullModel(model.name, model.n_views, model_keypoints, model_descriptor, model_cloud, model_pose, detector);
    //        //        dataset.loadModel(
    //        //                model.name,
    //        //                model.n_views,
    //        //                model_full_cloud,
    //        //                model_cloud,
    //        //                model_rgb,
    //        //                model_rgb_full,
    //        //                model_pose
    //        //                );
    //
    //        viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");
    //        viewer->addPointCloud(model_cloud, "cloud");
    //
    //        while (!viewer->wasStopped()) {
    //            viewer->spinOnce();
    //        }
    //    }
    //    dataset.loadModel(
    //            model.name,
    //            model.n_views,
    //            model_cloud,
    //            )


    return 0;
}
