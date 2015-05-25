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
#include "WillowDataset.h"
#include "PipeLine.h"

using namespace std;
using namespace BoldLib;



visy::Parameters* parameters;

/*
 *
 */
int
main(int argc, char** argv) {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    /** PARAMETERS */
    parameters = new visy::Parameters(argc, argv);
    parameters->putFloat("maxgc");
    parameters->putFloat("gc_size");
    parameters->putString("detector");
    parameters->putString("sizes");
    parameters->putString("name");
    parameters->putInt("nbin");
    parameters->putInt("occlusion");
    parameters->putBool("d");
    parameters->putFloat("down");
    parameters->putString("dataset");
    parameters->putInt("set");
    parameters->putInt("scene");
    parameters->putInt("untextured");
    parameters->putFloat("gt_distance");
    parameters->putFloat("f_th");
    parameters->putString("test_name");

    int use_occlusion = false;
    bool untextured = parameters->getInt("untextured")>=1;
    
    int min_gc = 3;
    int max_gc = parameters->getFloat("maxgc") > 0 ? parameters->getFloat("maxgc") : 40;

    /**P PIPE */
    visy::pipes::PipeParameters pipeParameters;
    pipeParameters.downsampling_leaf = parameters->getFloat("down");
    pipeParameters.hv_inlier_threshold = 0.02f;

    /** DATASET*/
    visy::dataset::WillowDataset dataset(parameters->getString("dataset"));


    /** DETECTOR CREATION */
    bool different_detectors = parameters->getBool("d");
    visy::detectors::Detector * detector;
    visy::detectors::Detector * detector_model;
    detector = visy::detectors::utils::buildDetectorFromString(parameters->getString("detector"), parameters);
    detector_model = visy::detectors::utils::buildDetectorFromString(parameters->getString("detector"), parameters, different_detectors);

    std::cout << "MODEL DETECTOR: " << detector_model->buildName() << std::endl;
    std::cout << "SCENE DETECTOR: " << detector->buildName() << std::endl;


    /** RESULTS SETS*/
    int gcs = max_gc + 1;
    std::vector<visy::dataset::PrecisionRecallRow> precisions(gcs);

    /** TEST NAME*/
    std::stringstream ss;
    ss << "test_";
    ss << parameters->getString("test_name")<<"_";
    ss << parameters->getFloat("gc_size") << "_" << parameters->getString("detector");

    std::string test_name = ss.str();


    std::cout << "Start Test: " << test_name << std::endl;

    for (int model_index = 0; model_index < dataset.models->size(); model_index++) {
        /**MODEL */
        visy::dataset::Model model = dataset.models->at(model_index);
        
        if(untextured && !model.no_texture)continue;
        
        std::vector<visy::extractors::KeyPoint3D> model_keypoints;
        cv::Mat model_descriptor;
        pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr model_cloud_filtered(new pcl::PointCloud<PointType>());
        cv::Mat model_rgb;
        Eigen::Matrix4f model_pose;

        std::cout << "Loading model: " << model.name << std::endl;

        dataset.fetchFullModel(model.name, model.n_views, model_keypoints, model_descriptor, model_cloud, model_pose, detector_model);
        std::cout << "Loaded model: " << model.name << " keypoints:" << model_keypoints.size() << std::endl;

        for (int set_index = 0; set_index < dataset.scenes->size(); set_index++) {
            /* LOAD SET */
            visy::dataset::SetScene set = dataset.scenes->at(set_index);


            for (int scene_index = 0; scene_index <= set.scene_number; scene_index++) {
                /**LOAD SCENE */
                std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
                cv::Mat scene_descriptor;
                cv::Mat scene_rgb, scene_rgb_full;
                pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr scene_cloud_filtered(new pcl::PointCloud<PointType>());
                std::vector<visy::dataset::Annotation> scene_annotations;

                dataset.loadScene(set.set_number, scene_index, scene_cloud, scene_rgb);
                dataset.loadAnnotiationsFromSceneFile(model.name, set.set_number, scene_index, scene_annotations);

                /** DETECTIOn */
                detector->detect(scene_rgb, scene_cloud, scene_keypoints, scene_descriptor);

                /* RECOGNITION PIPE */
                /* MATCHES */
                std::vector<cv::DMatch> matches;
                std::vector<cv::DMatch> good_matches;
                visy::tools::matchKeypointsBrute(matches, good_matches, model_descriptor, scene_descriptor);

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

                visy::tools::cloudDownsample(model_cloud, model_cloud_filtered, pipeParameters.downsampling_leaf);
                visy::tools::cloudDownsample(scene_cloud, scene_cloud_filtered, pipeParameters.downsampling_leaf);


                cv::Mat precisionMatHv = cv::Mat(gcs, 12, CV_32F, float(0));

                for (int gc_th = min_gc; gc_th <= max_gc; gc_th++) {

                    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
                    std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;
                    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > registered_rototranslations;
                    std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
                    std::vector<bool> hypotheses_mask; // Mask Vector to identify positive hypotheses
                    int hv_counter = 0;


                    /* GEOMETRU CONSISTENCY GROUPING*/
                    std::vector < pcl::Correspondences > clustered_corrs;
                    visy::extractors::utils::keypointsGeometricConsistencyGrouping(
                            parameters->getFloat("gc_size"),
                            gc_th,
                            matched_model_keypoints,
                            matched_scene_keypoints,
                            model_scene_corrs,
                            rototranslations,
                            clustered_corrs);

                    if (rototranslations.size() > 0) {

                        bool use_HV = false;
                        if (use_HV) {
                            /**
                             * Generates clouds for each instances found 
                             */
                            for (size_t i = 0; i < rototranslations.size(); ++i) {
                                pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType> ());
                                pcl::transformPointCloud(*model_cloud_filtered, *rotated_model, rototranslations.at(i));
                                instances.push_back(rotated_model);
                            }

                            /**
                             * ICP
                             */
                            visy::tools::registerInstances(
                                    scene_cloud_filtered,
                                    instances,
                                    registered_instances,
                                    registered_rototranslations,
                                    pipeParameters.icp_max_iterations,
                                    pipeParameters.icp_max_distance);

                            //                        registered_instances = instances;

                            if (registered_instances.size() > 0) {

                                /**
                                 * Hypothesis Verification
                                 */
                                visy::tools::hypothesesVerification(
                                        scene_cloud_filtered,
                                        registered_instances,
                                        hypotheses_mask,
                                        pipeParameters.hv_inlier_threshold,
                                        pipeParameters.hv_occlusion_threshold,
                                        pipeParameters.hv_regularizer,
                                        pipeParameters.hv_radius_clutter,
                                        pipeParameters.hv_clutter_regularizer,
                                        pipeParameters.hv_detect_clutter,
                                        pipeParameters.hv_radius_normal
                                        );
                            }
                        } else {
                            hypotheses_mask.resize(rototranslations.size());
                            std::fill(hypotheses_mask.begin(), hypotheses_mask.end(), false);
                            
                            for (int rs = 0; rs < rototranslations.size(); rs++) {
                                Eigen::Matrix4f hip_pose = rototranslations[rs] * model_pose;

                                for (int a = 0; a < scene_annotations.size(); a++) {
                                    cv::Point3f hip_position(hip_pose(0, 3), hip_pose(1, 3), hip_pose(2, 3));
                                    cv::Point3f gt_position(scene_annotations[a].pose(0, 3), scene_annotations[a].pose(1, 3), scene_annotations[a].pose(2, 3));
                                    cv::Point3f pos_dist_vector = hip_position - gt_position;
                                    float pos_dist = cv::norm(pos_dist_vector);

                                    if (pos_dist <= parameters->getFloat("gt_distance")) {
                                        hypotheses_mask[rs] = true;
                                    }

                                }
                            }
                        }

                        for (int hi = 0; hi < hypotheses_mask.size(); hi++) {
                            if (hypotheses_mask[hi]) {
                                hv_counter++;
                            }
                        }
                    }

                    int P = dataset.checkInstancesNumber(model.name, scene_annotations);

                    int N = registered_instances.size();
                    int TP = std::min((int) N, P);
                    int FP = N - P > 0 ? N - P : 0;
                    int FN = P - TP > 0 ? P - TP : 0;
                    int TN = (N <= 0 && P <= 0) ? 1 : 0;

                    int N_HV = hv_counter;
                    int TP_HV = std::min((int) N_HV, P);
                    int FP_HV = N_HV - P > 0 ? N_HV - P : 0;
                    int FN_HV = P - TP_HV > 0 ? P - TP_HV : 0;
                    int TN_HV = (N_HV <= 0 && P <= 0) ? 1 : 0;

                    std::cout << model.name << " -> " << set.set_number << "," << scene_index << " gc:" << gc_th << " ";
                    std::cout << " P: " << P;

                    std::cout << " N: " << N;
                    std::cout << " TP: " << TP;
                    std::cout << " FP: " << FP;
                    std::cout << " FN: " << FN;
                    std::cout << " TN: " << TN;

                    std::cout << " # " << N;
                    std::cout << " N_HV: " << N_HV;
                    std::cout << " TP: " << TP_HV;
                    std::cout << " FP: " << FP_HV;
                    std::cout << " FN: " << FN_HV;
                    std::cout << " TN: " << TN_HV;
                    std::cout << std::endl;

                    precisionMatHv.at<float>(gc_th, 0) = precisionMatHv.at<float>(gc_th, 0) + P;
                    precisionMatHv.at<float>(gc_th, 1) = precisionMatHv.at<float>(gc_th, 1) + N;
                    precisionMatHv.at<float>(gc_th, 2) = precisionMatHv.at<float>(gc_th, 2) + TP;
                    precisionMatHv.at<float>(gc_th, 3) = precisionMatHv.at<float>(gc_th, 3) + FP;
                    precisionMatHv.at<float>(gc_th, 4) = precisionMatHv.at<float>(gc_th, 4) + TN;
                    precisionMatHv.at<float>(gc_th, 5) = precisionMatHv.at<float>(gc_th, 5) + FN;
                    precisionMatHv.at<float>(gc_th, 6) = precisionMatHv.at<float>(gc_th, 6) + N_HV;
                    precisionMatHv.at<float>(gc_th, 7) = precisionMatHv.at<float>(gc_th, 7) + TP_HV;
                    precisionMatHv.at<float>(gc_th, 8) = precisionMatHv.at<float>(gc_th, 8) + FP_HV;
                    precisionMatHv.at<float>(gc_th, 9) = precisionMatHv.at<float>(gc_th, 9) + TN_HV;
                    precisionMatHv.at<float>(gc_th, 10) = precisionMatHv.at<float>(gc_th, 10) + FN_HV;

                }

                dataset.savePrecisionMat(test_name, precisionMatHv);

                //
                //          std::cout << model.name << " -> " << set.set_number << "," << scene_index << " gc:" << gc_th << " ";
                //          std::cout << " N: " << N;
                //          std::cout << " P: " << P;
                //          std::cout << " TP: " << TP;
                //          std::cout << " FP: " << FP;
                //          std::cout << " FN: " << FN;
                //          std::cout << " TN: " << TN;
                //          std::cout << std::endl;
                //


            }
            //        dataset.savePrecisionMat(test_name, precisionMat);

        }
    }


    //  std::vector<visy::extractors::KeyPoint3D> model_keypoints;
    //  cv::Mat model_descriptor;
    //  pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
    //  pcl::PointCloud<PointType>::Ptr model_full_cloud(new pcl::PointCloud<PointType>());
    //  cv::Mat model_rgb;
    //  Eigen::Matrix4f model_pose;
    //
    //  if (dataset.loadDescription(model.name, model_keypoints, model_descriptor, model_pose, detector->buildName()))
    //  {
    //    visy::dataset::IrosDataset::loadModel(model.name, 0, model_full_cloud, model_cloud, model_rgb, model_pose);
    //  }
    //  else
    //  {
    //    dataset.fetchFullModel(model.name, model.n_views, model_keypoints, model_descriptor, model_cloud, model_pose, detector);
    //
    //  }


    //  std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
    //  cv::Mat scene_descriptor;
    //  cv::Mat scene_rgb, scene_rgb_full;
    //  pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());
    //
    //  visy::dataset::IrosDataset::loadScene(parameters->getInt("set"), parameters->getInt("scene"), scene_cloud, scene_rgb);
    //
    //  detector->detect(scene_rgb, scene_cloud, scene_keypoints, scene_descriptor);
    //
    //
    //
    //
    //  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    //
    //  visy::extractors::utils::modelSceneMatch(
    //          model_keypoints,
    //          scene_keypoints,
    //          model_descriptor,
    //          scene_descriptor,
    //          rototranslations,
    //          parameters->getFloat("gc_size"),
    //          parameters->getFloat("gc_th")
    //          );
    //
    //
    //
    //
    //
    //  std::cout << "FOUND:" << rototranslations.size() << std::endl;
    //
    //
    //  for (int i = 0; i < rototranslations.size(); i++)
    //  {
    //    pcl::PointCloud<PointType>::Ptr model_projection(new pcl::PointCloud<PointType>());
    //    pcl::transformPointCloud(*model_cloud, *model_projection, rototranslations[i]);
    //    std::stringstream ss;
    //    ss << "Instance_" << i << "_cloud";
    //    visy::tools::displayCloud(*viewer, model_projection, 0, 255, 0, 3.0f, ss.str());
    //  }
    //
    //
    //
    //  viewer->addPointCloud(scene_cloud, "scene");
    //
    //
    //  //  visy::extractors::KeyPoint3D::draw3DKeyPoints3D(*viewer, model_keypoints, cv::Scalar(0, 255, 0), "ciao", true);
    //
    //  //  for (int i = 0; i < rototranslations.size(); i++)
    //  //  {
    //  //    pcl::PointCloud<PointType>::Ptr model_projection(new pcl::PointCloud<PointType>());
    //  //    pcl::transformPointCloud(*model_cloud, *model_projection, rototranslations[i]);
    //  //    std::stringstream ss;
    //  //    ss << "Instance_" << i << "_cloud";
    //  //    visy::tools::displayCloud(*viewer, model_projection, 0, 255, 0, 3.0f, ss.str());
    //  //  }
    //
    //  //  cv::namedWindow("out", cv::WINDOW_NORMAL);
    //  //  cv::imshow("out", out);
    //  //  cv::namedWindow("out_scene", cv::WINDOW_NORMAL);
    //  //  cv::imshow("out_scene", out_scene);
    //
    //  while (!viewer->wasStopped())
    //  {
    //    cv::waitKey(100);
    //    viewer->spinOnce();
    //  }



    return 1;
}
