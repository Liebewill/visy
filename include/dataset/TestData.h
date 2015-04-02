/* 
 * File:   TestData.h
 * Author: daniele
 *
 * Created on March 31, 2015, 6:59 PM
 */

#ifndef TESTDATA_H
#define	TESTDATA_H

#include "extractors/KeyPoint3D.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl-1.7/pcl/common/io.h>
#include <pcl-1.7/pcl/common/transforms.h>
#include "Detector.h"

namespace visy {
    namespace dataset {
        
         struct PrecisionRecallRow {
            int GC = -1;
            int P = 0;
            int N = 0;
            int TP = 0;
            int FP = 0;
            int FN = 0;
            int TN = 0;
        };
        
        struct Annotation {
            std::string model_name;
            int set_index;
            int scene_index;
            Eigen::Matrix4f pose;
            float occlusion;
        };

        struct Hit {
            bool bad = false;
            int ground_index;
            Annotation ground;
            Eigen::Matrix4f instance_pose;
            float distance_error;
            float rotation_error;
        };

        struct Model {
            std::string name;
            int n_views;

            Model(std::string name, int n_views) {
                this->name = name;
                this->n_views = n_views;
            }

        };

        struct SetScene {
            int set_number;
            int scene_number;

            SetScene(int set, int scene) {
                set_number = set;
                scene_number = scene;
            }
        };

        struct ResultSceneView {
            std::string model_name = "";
            int set_number = 0;
            int scene_number = 0;
            int gc_th = 0;
            int found = 0;
            int found_hv = 0;
            int gt = 0;
            int p = 0;
            int tp = 0;
            int fp = 0;
            int fn = 0;
            int tn = 0;

            std::string toString() {
                std::stringstream ss;
                ss << "Model:" << model_name << ";";
                ss << "Set:" << set_number << ";";
                ss << "Scene:" << scene_number << ";";
                ss << "GC:" << gc_th << ";";
                ss << "HV:" << found_hv << ";";
                ss << "Gt:" << gt << ";";
                ss << "Found:" << found << ";";
                ss << "P:" << p << ";";
                ss << "TP:" << tp << ";";
                ss << "FP:" << fp << ";";
                ss << "FN:" << fn << ";";
                ss << "TN:" << tn << ";";
                return ss.str();
            }
        };

        struct ResultScene {
            int set_number = 0;
            int gc_th = 0;
            int found_hv = 0;
            int p = 0;
            int tp = 0;
            int fp = 0;
            int fn = 0;
            int tn = 0;
            float precision = 0;
            float recall = 0;

            std::string toString() {
                std::stringstream ss;
                ss << set_number << ";";
                ss << gc_th << ";";
                ss << found_hv << ";";
                ss << p << ";";
                ss << tp << ";";
                ss << fp << ";";
                ss << fn << ";";
                ss << tn << ";";
                ss << precision << ";";
                ss << recall << ";";
                return ss.str();
            }
        };

        class TestData  {
        public:

            static std::string BASE_PATH;
            static std::string TRAINING_PATH;
            static std::string SETS_PATH;
            static std::string ANNOTATION_PATH;
            static std::vector<Model>* models;
            static std::vector<SetScene>* scenes;

            TestData();
            virtual ~TestData();
            static void init();
            static void loadIndices(std::string path, std::vector<int>& indices);
            static void loadModel(std::string simpleName, int view_number, pcl::PointCloud<PointType>::Ptr full_model, pcl::PointCloud<PointType>::Ptr model, cv::Mat& rgb_model, Eigen::Matrix4f& pose);
            static void loadModel(std::string simpleName, int view_number, pcl::PointCloud<PointType>::Ptr full_model, pcl::PointCloud<PointType>::Ptr model, cv::Mat& rgb_model, cv::Mat& rgb_full_model, Eigen::Matrix4f& pose);
            
            static Model findModelByName(std::string simpleName);
            
            static void loadScene(int set_number, int scene_number, pcl::PointCloud<PointType>::Ptr scene, cv::Mat& rgb_scene);
            static std::string paddedNumber(int number, int length, bool after = false);
            static void loadPoseFromFile(std::string path, Eigen::Matrix4f& pose);
            static void loadPoseFromSceneFile(std::string model_name, int set_number, int scene_number, int model_index, Eigen::Matrix4f& pose);
            static void loadAnnotiationsFromSceneFile(std::string model_name, int set_number, int scene_number, std::vector<Annotation>& annotations);
            static void loadFullAnnotiationsFromSceneFile(int set_number, int scene_number, std::vector<Annotation>& annotations);
            static int checkInstancesNumber(std::string model_name, std::vector<Annotation>& annotations);
            static bool checkHV(std::string model_name, Eigen::Matrix4f pose, std::vector<Annotation>& annotations, float distance_th, float rot_th);
            static bool checkPoses(std::string model_name, std::vector<Eigen::Matrix4f>& poses, std::vector<Annotation>& annotations);

            void fetchFullModel(std::string model_name, int views_max_number, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, pcl::PointCloud<PointType>::Ptr& cloud,Eigen::Matrix4f& reference_pose, visy::detectors::Detector * detector);
            void fetchFullModelSimple(std::string model_name, int views_max_number, std::vector<visy::extractors::KeyPoint3D>& keypoints,  pcl::PointCloud<PointType>::Ptr& cloud,Eigen::Matrix4f& reference_pose, visy::detectors::Detector * detector);
        private:

        };
    }
}



#endif	/* TESTDATA_H */

