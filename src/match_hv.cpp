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
    parameters->putInt("scene");
    parameters->putInt("nbin");
    parameters->putInt("occlusion");
    parameters->putInt("hough");
    parameters->putFloat("f_th");
    parameters->putFloat("gt_distance");

    /**VIEWER*/
    pcl::visualization::PCLVisualizer * viewer;
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
    dataset.loadScene(set_number, scene_number, scene_cloud, scene_rgb);
    dataset.loadAnnotiationsFromSceneFile(model.name, set_number, scene_number, annotations);





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
    std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > registered_rototranslations;
    std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;

    visy::pipes::PipeParameters pipeParameters;
    visy::pipes::PipeLine pipe(detector, pipeParameters);

    /** PIPE TRAIN */
    pipe.pipeParameters.downsampling_leaf = parameters->getFloat("down");
    pipe.pipeParameters.gc_size = parameters->getFloat("gc_size");
    pipe.pipeParameters.gc_th = parameters->getFloat("gc_th");
    bool use_hough = parameters->getInt("hough")>=1;
    pipe.pipeParameters.use_hough = use_hough;
    
    pipe.train(
            model_cloud,
            scene_cloud,
            model_keypoints,
            scene_keypoints,
            model_descriptor,
            scene_descriptor,
            model_cloud_filtered,
            scene_cloud_filtered);

    std::vector<bool> hypotheses_mask(pipe.transforms.size()); // Mask Vector to identify positive hypotheses
    std::fill(hypotheses_mask.begin(), hypotheses_mask.end(), false);

    /** PIPE RUN*/

    //        pipe.run(
    //                instances,
    //                registered_instances,
    //                rototranslations,
    //                registered_rototranslations,
    //                hypotheses_mask);

    /**
     * Generates clouds for each instances found 
     */
    for (size_t i = 0; i < pipe.transforms.size(); ++i) {
        pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType> ());
        pcl::transformPointCloud(*model_cloud_filtered, *rotated_model, pipe.transforms.at(i));
        instances.push_back(rotated_model);
    }

    for (int i = 0; i < pipe.transforms.size(); i++) {
        Eigen::Matrix4f hip_pose = pipe.transforms[i] * model_pose;

        for (int a = 0; a < annotations.size(); a++) {
            std::cout << annotations[a].model_name << " - " << annotations[a].occlusion << std::endl;
            cv::Point3f hip_position(hip_pose(0, 3), hip_pose(1, 3), hip_pose(2, 3));
            cv::Point3f gt_position(annotations[a].pose(0, 3), annotations[a].pose(1, 3), annotations[a].pose(2, 3));
            cv::Point3f pos_dist_vector = hip_position - gt_position;
            float pos_dist = cv::norm(pos_dist_vector);


            if (pos_dist <= parameters->getFloat("gt_distance")) {
                hypotheses_mask[i] = true;
            }

        }
    }

    /** DISPLAY */

    viewer->addPointCloud(scene_cloud_filtered, "scene");

    for (int i = 0; i < hypotheses_mask.size(); i++) {
        std::stringstream ss;
        ss << "hip_" << i;
        if (hypotheses_mask[i]) {
            visy::tools::displayCloud(*viewer, instances[i], 0, 255, 0, 3.0f, ss.str());
        } else {
            //            visy::tools::displayCloud(*viewer, instances[i], 255, 0, 0, 0.5f, ss.str());
        }
    }
    //        std::stringstream ss;
    //        ss << "Instance_" << i << "_cloud";
    //        if (hypotheses_mask[i]) {
    //
    //            visy::tools::displayCloud(*viewer, instances[i], 0, 255, 255, 3.0f, ss.str());
    //            ss << "_reg";
    //            visy::tools::displayCloud(*viewer, registered_instances[i], 0, 255, 0, 3.0f, ss.str());
    //
    //            Eigen::Matrix4f p = rototranslations[i] * model_pose;
    //            std::cout << "Good:\n" << p << std::endl;
    //        } else {
    //
    //            visy::tools::displayCloud(*viewer, registered_instances[i], 255, 0, 0, 3.0f, ss.str());
    //        }
    //    }

    for (int a = 0; a < annotations.size(); a++) {
        std::cout << annotations[a].model_name << "\n" << annotations[a].pose << std::endl;
    }
    float in = dataset.checkInstancesNumber(model.name, annotations);
    std::cout << "Searching for: " << in << " instances " << std::endl;

    while (!viewer->wasStopped()) {
        cv::waitKey(100);
        viewer->spinOnce();
    }

    return 1;
}
