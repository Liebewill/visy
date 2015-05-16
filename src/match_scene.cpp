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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


#include <pcl/impl/point_types.hpp>
#include <pcl/common/centroid.h>


#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/common/transforms.h>

#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <sstream>
#include <string>
#include <fstream>

#include <tools.h>
#include <opencv2/core/mat.hpp>

#include "Extractor.h"
#include "Bold3DExtractor.h"
#include "extractors/extrators_utils.h"
#include "Descriptor.h"
#include "Bold3DDescriptorMultiBunch.h"
#include "Detector.h"
#include "Bold3DMDetector.h"
#include "Bold3DRDetector.h"
#include "BoldDetector.h"
#include "Parameters.h"
#include "Bold3DM2Detector.h"
#include "Bold3DR2Detector.h"
#include "Bold3DM2MultiDetector.h"
#include "detectors/detectors_utils.h"
#include "commons/commons.h"
#include "WillowDataset.h"
#include "PipeLine.h"

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

visy::Parameters* parameters;

pcl::visualization::PCLVisualizer * viewer;

cv::Mat scene_descriptor, scene_2_descriptor;
cv::Mat scene_rgb, scene_rgb_full;
cv::Mat scene_2_rgb, scene_2_rgb_full;
pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr scene_cloud_filtered(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr scene_2_cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr scene_2_cloud_filtered(new pcl::PointCloud<PointType>());

std::vector<visy::extractors::KeyPoint3D> scene_keypoints, scene_2_keypoints;
std::vector<visy::extractors::KeyPoint3D> scene_keypoints_seletected;
std::vector<visy::extractors::KeyPoint3D> scene_2_keypoints_seletected;
std::vector<cv::DMatch> matches;
std::vector<cv::DMatch> good_matches;
int scene_keypoint_selected_index;

cv::Mat out, out2;

void
redraw() {
    out = scene_rgb.clone();
    out2 = scene_2_rgb.clone();

    visy::extractors::KeyPoint3D::draw3DKeyPoints(out, scene_keypoints, cv::Scalar(0, 0, 255), 1.0f,true);
    visy::extractors::KeyPoint3D::draw3DKeyPoints(out, scene_keypoints_seletected, cv::Scalar(0, 255, 0), 1.0f,true);
    visy::extractors::KeyPoint3D::draw3DKeyPoints(out2, scene_2_keypoints, cv::Scalar(0, 0, 255), 1.0f,true);
    visy::extractors::KeyPoint3D::draw3DKeyPoints(out2, scene_2_keypoints_seletected, cv::Scalar(0, 255, 0), 1.0f,true);

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addPointCloud(scene_cloud, "scene");
    //    draw3DKeyPointsColor(*viewer, out, scene_keypoints, cv::Scalar(0, 255, 0), "scene_kps", true);
    cv::imshow("out", out);
    cv::imshow("out2", out2);
}

void
CallBackFunc(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        std::cout << "Click " << x << std::endl;
        float min_dist = 1000.0f;
        int min_index = -1;

        for (int i = 0; i < scene_keypoints.size(); i++) {
            cv::Point2f d = scene_keypoints[i].pt - cv::Point2f(x, y);
            float dd = cv::norm(d);
            if (dd < min_dist) {
                min_dist = dd;
                min_index = i;
            }
        }

        if (min_index >= 0) {
            scene_keypoint_selected_index = min_index;
            scene_keypoints_seletected.clear();
            scene_2_keypoints_seletected.clear();
            scene_keypoints_seletected.push_back(scene_keypoints[min_index]);
            
            
            
            for(int i = 0; i < matches.size(); i++){
                if(matches[i].trainIdx==min_index){
                    scene_2_keypoints_seletected.push_back(scene_2_keypoints[matches[i].queryIdx]);
                }
            }
            
            cv::Point3f p1 = scene_keypoints[min_index].pt3D_1;
            cv::Point3f p2 = scene_keypoints[min_index].pt3D_2;
            float d3d =cv::norm(p2 - p1);
            std::cout << "3D Size: " << d3d<<std::endl;
            
        } else {
            scene_keypoint_selected_index = -1;
        }
        redraw();
    }
}

int myrandom (int i) { return std::rand()%i;}

/*
 *
 */
int
main(int argc, char** argv) {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    /** PARAMETERS */
    parameters = new visy::Parameters(argc, argv);
    parameters->putFloat("gc_th");
    parameters->putFloat("gc_size");
    parameters->putFloat("down");
    parameters->putString("dataset");
    parameters->putString("detector");
    parameters->putString("model");
    parameters->putString("sizes");
    parameters->putInt("set");
    parameters->putInt("set2");
    parameters->putInt("scene");
    parameters->putInt("scene2");
    parameters->putInt("nbin");
    parameters->putInt("occlusion");
    parameters->putFloat("f_th");
    parameters->putFloat("gt_distance");


    /**VIEWER*/

    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");


    /** DETECTOR */
    visy::detectors::Detector * detector;
    visy::detectors::Detector * detector_model;

    detector = visy::detectors::utils::buildDetectorFromString(parameters->getString("detector"), parameters, false);
    detector_model = visy::detectors::utils::buildDetectorFromString(parameters->getString("detector"), parameters, true);

    std::cout << "Detector: " << detector->buildName() << std::endl;
    if (parameters->getString("detector") == "BOLD") {
        std::cout << "Descriptor size: " << 144 << std::endl;

    } else {
        if (detector->descriptor != NULL)
            std::cout << "Descriptor size: " << detector->descriptor->dfunction->getDataSize() << std::endl;

    }

    /** DATASET */
    std::cout << "Building Dataset: " << parameters->getString("dataset") << std::endl;
    visy::dataset::WillowDataset dataset(parameters->getString("dataset"));


    /** SCENE */



    int set_number = parameters->getInt("set");
    int scene_number = parameters->getInt("scene");
    std::vector<visy::dataset::Annotation> annotations;
    dataset.loadScene(set_number, scene_number, scene_cloud, scene_rgb);

    int set_2_number = parameters->getInt("set2");
    int scene_2_number = parameters->getInt("scene2");
    std::vector<visy::dataset::Annotation> annotations_2;
    dataset.loadScene(set_2_number, scene_2_number, scene_2_cloud, scene_2_rgb);




    /** DETECTION */
    detector->detect(scene_rgb, scene_cloud, scene_keypoints, scene_descriptor);
    detector->detect(scene_2_rgb, scene_2_cloud, scene_2_keypoints, scene_2_descriptor);
    
    std::cout << "Scene kps: " << scene_keypoints.size() << std::endl;
    std::cout << "Scene 2 kps: " << scene_2_keypoints.size() << std::endl;


    /** MODEL-SCENE MATCH */

    /* MATCHES */

    visy::tools::matchKeypointsBrute(matches, good_matches, scene_descriptor, scene_2_descriptor);
    std::cout << "Matches: "<<matches.size()<<std::endl;
    
    cv::namedWindow("out", cv::WINDOW_NORMAL);
    cv::namedWindow("out2", cv::WINDOW_NORMAL);
    cv::setMouseCallback("out", CallBackFunc);

    redraw();


    /** DISPLAY */


    while (!viewer->wasStopped()) {
        cv::waitKey(100);
        viewer->spinOnce();
    }

    return 1;
}
