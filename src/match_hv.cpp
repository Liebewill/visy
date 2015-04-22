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
    parameters->putString("detector");
    parameters->putString("model");
    parameters->putString("sizes");
    parameters->putInt("set");
    parameters->putInt("scene");
    parameters->putInt("nbin");
    parameters->putInt("occlusion");

    /**VIEWER*/
    pcl::visualization::PCLVisualizer * viewer;
    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");


    /** DETECTOR */
    visy::detectors::Detector * detector;
    visy::detectors::Detector * detector_model;

    detector = visy::detectors::utils::buildDetectorFromString(parameters->getString("detector"), parameters, false);
    detector_model = visy::detectors::utils::buildDetectorFromString(parameters->getString("detector"), parameters, true);

    std::cout << "Detector: " << detector->buildName() << std::endl;


    /** DATASET */
    visy::dataset::WillowDataset dataset;
    dataset.init();
    
    /** MODEL */
    visy::dataset::Model model = dataset.findModelByName(parameters->getString("model"));
    std::vector<visy::extractors::KeyPoint3D> model_keypoints;
    cv::Mat model_descriptor;
    pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr model_cloud_filtered(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr model_full_cloud(new pcl::PointCloud<PointType>());
    cv::Mat model_rgb;
    Eigen::Matrix4f model_pose;

    dataset.fetchFullModel(model.name, model.n_views, model_keypoints, model_descriptor, model_cloud, model_pose, detector_model);

    /** SCENE */
    std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
    cv::Mat scene_descriptor;
    cv::Mat scene_rgb, scene_rgb_full;
    pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr scene_cloud_filtered(new pcl::PointCloud<PointType>());

    int set_number = parameters->getInt("set");
    int scene_number = parameters->getInt("scene");
    std::vector<visy::dataset::Annotation> annotations;
    dataset.loadScene(set_number,scene_number, scene_cloud, scene_rgb);
    dataset.loadAnnotiationsFromSceneFile(model.name,set_number,scene_number,annotations);
    
    
    
    
    
    /** DETECTION */
    boost::posix_time::ptime time_start(boost::posix_time::microsec_clock::local_time());
    detector->detect(scene_rgb, scene_cloud, scene_keypoints, scene_descriptor);
    std::cout << "Model kps: " << model_keypoints.size() << std::endl;
    std::cout << "Scene kps: " << scene_keypoints.size() << std::endl;
    boost::posix_time::ptime time_end(boost::posix_time::microsec_clock::local_time());
    boost::posix_time::time_duration duration(time_end - time_start);
    cout << "Detector time: " << duration << '\n';

    /** MODEL-SCENE MATCH */
    
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;

    visy::extractors::utils::modelSceneMatch(
            model_keypoints,
            scene_keypoints,
            model_descriptor,
            scene_descriptor,
            rototranslations,
            parameters->getFloat("gc_size"),
            parameters->getFloat("gc_th")
            );

    std::cout << "FOUND:" << rototranslations.size() << std::endl;
    
    /**
     * Downsampling Model Cloud
     */
    visy::tools::cloudDownsample(model_cloud,model_cloud_filtered,parameters->getFloat("down"));
    visy::tools::cloudDownsample(scene_cloud,scene_cloud_filtered,parameters->getFloat("down"));
    viewer->addPointCloud(scene_cloud_filtered, "scene");
    
    /**
     * Generates clouds for each instances found 
     */
    std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;

    for (size_t i = 0; i < rototranslations.size(); ++i) {
        pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType> ());
        pcl::transformPointCloud(*model_cloud_filtered, *rotated_model, rototranslations[i]);
        instances.push_back(rotated_model);
    }

    /**
     * ICP
     */
    std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
    visy::tools::registerInstances(scene_cloud_filtered,instances,registered_instances,5,0.005f);
    

    /**
     * Hypothesis Verification
     */
    cout << "--- Hypotheses Verification ---" << endl;
    std::vector<bool> hypotheses_mask; // Mask Vector to identify positive hypotheses
    visy::tools::hypothesesVerification(scene_cloud_filtered,registered_instances,hypotheses_mask,0.015f);

    for (int i = 0; i < hypotheses_mask.size(); i++) {
        if (hypotheses_mask[i]) {
            cout << "Instance " << i << " is GOOD! <---" << endl;
        }
    }
    cout << "-------------------------------" << endl;

    
    /** DISPLAY */
    
    for (int i = 0; i < registered_instances.size(); i++) {
        std::stringstream ss;
        ss << "Instance_" << i << "_cloud";
        if (hypotheses_mask[i]) {

            visy::tools::displayCloud(*viewer, registered_instances[i], 0, 255, 0, 3.0f, ss.str());
            
            Eigen::Matrix4f p = model_pose*rototranslations[i];
            std::cout << "Good:\n"<<p<<std::endl;
        } else {

            visy::tools::displayCloud(*viewer, registered_instances[i], 255, 0, 0, 3.0f, ss.str());
        }
    }
    
    for(int a = 0; a < annotations.size(); a++){
        std::cout << annotations[a].model_name<<"\n"<<annotations[a].pose<<std::endl;
    }

    while (!viewer->wasStopped()) {
        cv::waitKey(100);
        viewer->spinOnce();
    }

    return 1;
}
