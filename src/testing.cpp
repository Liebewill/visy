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

#include "Parameters.h"
#include "TestData.h"

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

/*
 *
 */
int
main(int argc, char** argv) {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (pcl::io::loadPCDFile("/home/daniele/iros_dataset/test_set/set_00005/00001.pcd", *scene) < 0) {
        std::cout << "Error loading cloud." << std::endl;
    }

    pcl::visualization::PCLVisualizer * viewer;
    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");

    viewer->addPointCloud(scene, "scene2");
    
    while(!viewer->wasStopped()){
        viewer->spinOnce();
    }
    //  visy::extractors::KeyPoint3D kp3d;

//    visy::dataset::TestData dataset;
    std::cout << "OK" << std::endl;
    return 0;
}
