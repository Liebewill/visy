/* 
 * File:   WillowDataset.cpp
 * Author: daniele
 * 
 * Created on April 22, 2015, 10:16 AM
 */

#include "WillowDataset.h"
namespace visy {
    namespace dataset {

        WillowDataset::WillowDataset(std::string name) {
            this->name = name;
            this->init();
        }

        WillowDataset::~WillowDataset() {
        }

        void WillowDataset::init() {

            this->models = new std::vector<Model>();
            this->scenes = new std::vector<SetScene>();

            if (this->name == "WILLOW") {
                this->BASE_PATH = "/home/daniele/work/data/willow_dataset/";
                this->TRAINING_PATH = "training_data/";
                this->SETS_PATH = "test_set/";
                this->SET_PATTERN_NAME = "T_$$_willow_dataset";
                this->SCENE_PATTERN_NAME = "cloud_$$$$$$$$$$";
                this->ANNOTATION_PATH = "annotations/";

                WillowDataset::models->push_back(Model("object_01", 36));
                WillowDataset::models->push_back(Model("object_02", 36));
                WillowDataset::models->push_back(Model("object_03", 36));
                WillowDataset::models->push_back(Model("object_04", 36));
                WillowDataset::models->push_back(Model("object_05", 36));
                WillowDataset::models->push_back(Model("object_06", 36));
                WillowDataset::models->push_back(Model("object_07", 36));
                WillowDataset::models->push_back(Model("object_08", 36));
                WillowDataset::models->push_back(Model("object_09", 36));

                WillowDataset::models->push_back(Model("object_10", 36));
                WillowDataset::models->push_back(Model("object_11", 36));
                WillowDataset::models->push_back(Model("object_12", 36));
                WillowDataset::models->push_back(Model("object_13", 36));
                WillowDataset::models->push_back(Model("object_14", 36));
                WillowDataset::models->push_back(Model("object_15", 36));
                WillowDataset::models->push_back(Model("object_16", 36));
                WillowDataset::models->push_back(Model("object_17", 36));
                WillowDataset::models->push_back(Model("object_18", 36));
                WillowDataset::models->push_back(Model("object_19", 36));

                WillowDataset::models->push_back(Model("object_20", 36));
                WillowDataset::models->push_back(Model("object_21", 36));
                WillowDataset::models->push_back(Model("object_22", 36));
                WillowDataset::models->push_back(Model("object_23", 36));
                WillowDataset::models->push_back(Model("object_24", 36));
                WillowDataset::models->push_back(Model("object_25", 36));
                WillowDataset::models->push_back(Model("object_26", 36));
                WillowDataset::models->push_back(Model("object_27", 36));
                WillowDataset::models->push_back(Model("object_28", 36));
                WillowDataset::models->push_back(Model("object_29", 36));


                WillowDataset::models->push_back(Model("object_30", 36));
                WillowDataset::models->push_back(Model("object_31", 36));
                WillowDataset::models->push_back(Model("object_32", 36));
                WillowDataset::models->push_back(Model("object_33", 36));
                WillowDataset::models->push_back(Model("object_34", 36));
                WillowDataset::models->push_back(Model("object_35", 36));

                //SCENES
                WillowDataset::scenes->push_back(SetScene(1, 15));
                WillowDataset::scenes->push_back(SetScene(2, 15));
                WillowDataset::scenes->push_back(SetScene(3, 13));
                WillowDataset::scenes->push_back(SetScene(4, 13));
                WillowDataset::scenes->push_back(SetScene(5, 11));
                WillowDataset::scenes->push_back(SetScene(6, 13));
                WillowDataset::scenes->push_back(SetScene(7, 18));
                WillowDataset::scenes->push_back(SetScene(8, 14));
                WillowDataset::scenes->push_back(SetScene(9, 13));
                WillowDataset::scenes->push_back(SetScene(10, 13));
                WillowDataset::scenes->push_back(SetScene(11, 16));
                WillowDataset::scenes->push_back(SetScene(12, 14));
                WillowDataset::scenes->push_back(SetScene(13, 14));
                WillowDataset::scenes->push_back(SetScene(14, 14));
                WillowDataset::scenes->push_back(SetScene(15, 15));
                WillowDataset::scenes->push_back(SetScene(16, 14));
                WillowDataset::scenes->push_back(SetScene(17, 14));
                WillowDataset::scenes->push_back(SetScene(18, 13));
                WillowDataset::scenes->push_back(SetScene(19, 13));
                WillowDataset::scenes->push_back(SetScene(20, 14));
                WillowDataset::scenes->push_back(SetScene(21, 10));
                WillowDataset::scenes->push_back(SetScene(22, 12));
                WillowDataset::scenes->push_back(SetScene(23, 13));
                WillowDataset::scenes->push_back(SetScene(24, 15));
            } else if (this->name == "TW") {
                this->BASE_PATH = "/home/daniele/work/data/iros_dataset/";
                this->TRAINING_PATH = "training_data/";
                this->SETS_PATH = "test_set/";
                this->SET_PATTERN_NAME = "set_$$$$$";
                this->SCENE_PATTERN_NAME = "$$$$$";
                this->ANNOTATION_PATH = "annotations/";

                WillowDataset::models->push_back(Model("asus_box", 37));
                WillowDataset::models->push_back(Model("burti", 60));
                WillowDataset::models->push_back(Model("canon_camera_bag", 38));
                WillowDataset::models->push_back(Model("cisco_phone", 19));
                WillowDataset::models->push_back(Model("coffee_container", 16));
                WillowDataset::models->push_back(Model("felix_ketchup", 20));
                WillowDataset::models->push_back(Model("fruchtmolke", 49));
                WillowDataset::models->push_back(Model("jasmine_green_tea", 41));
                WillowDataset::models->push_back(Model("muller_milch_banana", 20));
                WillowDataset::models->push_back(Model("muller_milch_shoko", 19));
                WillowDataset::models->push_back(Model("opencv_book", 12));
                WillowDataset::models->push_back(Model("red_mug_white_spots", 79));
                WillowDataset::models->push_back(Model("skull", 39));
                WillowDataset::models->push_back(Model("strands_mounting_unit", 56));
                WillowDataset::models->push_back(Model("toilet_paper", 16));
                WillowDataset::models->push_back(Model("water_boiler", 19));
                WillowDataset::models->push_back(Model("yellow_toy_car", 19));

                //SCENES
                WillowDataset::scenes->push_back(SetScene(1, 6));
                WillowDataset::scenes->push_back(SetScene(2, 6));
                WillowDataset::scenes->push_back(SetScene(3, 13));
                WillowDataset::scenes->push_back(SetScene(4, 15));
                WillowDataset::scenes->push_back(SetScene(5, 9));
                WillowDataset::scenes->push_back(SetScene(6, 12));
                WillowDataset::scenes->push_back(SetScene(7, 7));
                WillowDataset::scenes->push_back(SetScene(8, 7));
                WillowDataset::scenes->push_back(SetScene(9, 8));
                WillowDataset::scenes->push_back(SetScene(10, 6));
                WillowDataset::scenes->push_back(SetScene(11, 8));
                WillowDataset::scenes->push_back(SetScene(12, 7));
                WillowDataset::scenes->push_back(SetScene(13, 17));
                WillowDataset::scenes->push_back(SetScene(14, 15));
                WillowDataset::scenes->push_back(SetScene(15, 12));
            }
        }

        Model WillowDataset::findModelByName(std::string simpleName) {
            for (int i = 0; i < WillowDataset::models->size(); i++) {
                if (WillowDataset::models->at(i).name == simpleName) {
                    return WillowDataset::models->at(i);
                }
            }
            return Model("INVALID", 0);
        }

        void WillowDataset::loadModel(std::string simpleName, int view_number, pcl::PointCloud<PointType>::Ptr full_model, pcl::PointCloud<PointType>::Ptr model, cv::Mat& rgb_model, cv::Mat& rgb_full_model, Eigen::Matrix4f& pose) {
            std::stringstream path;
            path << BASE_PATH << TRAINING_PATH << simpleName << ".pcd/";

            std::stringstream cloud_filename;
            cloud_filename << path.str() << "cloud_" << paddedNumber(view_number, 8) << ".pcd";
            std::stringstream indices_filename;
            indices_filename << path.str() << "object_indices_" << paddedNumber(view_number, 8) << ".pcd.ply";

            if (pcl::io::loadPCDFile(cloud_filename.str(), *full_model) < 0) {
                std::cout << "Error loading cloud." << std::endl;
            }


            std::vector<int> indices;
            loadIndices(indices_filename.str(), indices);

            visy::tools::rgbFromCloud(full_model, rgb_model, indices);
            pcl::copyPointCloud(*full_model, indices, *model);


            //LOAD POSE
            std::stringstream pose_path;
            pose_path << path.str() << "pose_" << paddedNumber(view_number, 8) << ".txt";
            loadPoseFromFile(pose_path.str(), pose);
        }

        std::string WillowDataset::paddedNumber(int number, int length, bool after) {
            std::stringstream id;
            id << number;
            int pad = length;
            pad = pad - id.str().length();
            id.str("");
            for (int i = 0; i < pad; i++) {
                id << "0";
            }
            id << number;
            return id.str();
        }

        void WillowDataset::loadIndices(std::string path, std::vector<int>& indices) {
            std::ifstream infile(path.c_str());
            std::string line;
            std::string key;
            int count = -1;
            int i = 0;
            bool is_data = false;
            int* idxs;
            while (std::getline(infile, line)) {
                std::istringstream iss(line);

                if (is_data) {
                    iss >> idxs[i];
                    indices.push_back(idxs[i]);
                    i++;
                    if (i == count - 1) {
                        break;
                    }
                } else {
                    iss>>key;

                    if (key == "element") {
                        iss>>key;
                        if (key == "vertex") {
                            iss>>count;
                            idxs = new int[count];
                            //std::cout << "Reading size:" << count << std::endl;
                        }
                    }

                    if (key == "end_header") {
                        is_data = true;
                    }
                }
            }
            delete[] idxs;
        }

        void WillowDataset::loadPoseFromFile(std::string path, Eigen::Matrix4f& pose) {
            std::ifstream infile(path.c_str());

            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    infile >> pose(i, j);
                }
            }
        }

        void WillowDataset::fetchFullModelSimple(std::string model_name, int views_max_number, std::vector<visy::extractors::KeyPoint3D>& keypoints, pcl::PointCloud<PointType>::Ptr& cloud, Eigen::Matrix4f& reference_pose, visy::detectors::Detector* detector) {
            std::vector<visy::extractors::KeyPoint3D> keypoints_temp;

            keypoints_temp.clear();

            for (int i = 0; i <= views_max_number; i++) {
                cv::Mat model_rgb, model_rgb_full;
                pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr model_cloud_full(new pcl::PointCloud<PointType>());
                Eigen::Matrix4f model_pose;

                std::vector<visy::extractors::KeyPoint3D> view_keypoints;
                std::vector<visy::extractors::KeyPoint3D> view_keypoints_rotated;

                //LOAD MODEL
                loadModel(
                        model_name, i,
                        model_cloud_full, model_cloud,
                        model_rgb, model_rgb_full,
                        model_pose);

                detector->extractor->extract(model_rgb, model_cloud_full, view_keypoints);

                if (view_keypoints.size() == 0)continue;

                if (keypoints_temp.size() == 0) {
                    cloud = model_cloud;
                    reference_pose = visy::tools::invertTransformationMatrix(model_pose);
                    view_keypoints_rotated = view_keypoints;

                } else {
                    Eigen::Matrix4f inv = reference_pose*model_pose;
                    //          view_keypoints_rotated = view_keypoints;
                    visy::extractors::KeyPoint3D::transformKeyPoint3Ds(view_keypoints, view_keypoints_rotated, inv);
                    pcl::transformPointCloud(*model_cloud, *model_cloud, inv);
                    cloud->points.insert(cloud->points.end(), model_cloud->points.begin(), model_cloud->points.end());
                }

                keypoints_temp.insert(keypoints_temp.end(), view_keypoints_rotated.begin(), view_keypoints_rotated.end());

            }

            detector->refineKeyPoints3D(keypoints_temp, keypoints);
        }

        void WillowDataset::fetchFullModel(std::string model_name, int views_max_number, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, pcl::PointCloud<PointType>::Ptr& cloud, Eigen::Matrix4f& reference_pose, visy::detectors::Detector* detector) {
            if (detector->isDetectorEmbedded()) {
                std::vector<visy::extractors::KeyPoint3D> keypoints_temp;
                cv::Mat descriptor_temp;

                keypoints_temp.clear();

                for (int i = 0; i <= views_max_number; i++) {
                    cv::Mat model_rgb, model_rgb_full;
                    pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
                    pcl::PointCloud<PointType>::Ptr model_cloud_full(new pcl::PointCloud<PointType>());
                    Eigen::Matrix4f model_pose;

                    std::vector<visy::extractors::KeyPoint3D> view_keypoints;
                    std::vector<visy::extractors::KeyPoint3D> view_keypoints_rotated;
                    cv::Mat view_descriptor;

                    //LOAD MODEL
                    loadModel(
                            model_name, i,
                            model_cloud_full, model_cloud,
                            model_rgb, model_rgb_full,
                            model_pose);

                    detector->detect(model_rgb, model_cloud_full, view_keypoints, view_descriptor);

                    if (view_keypoints.size() == 0)continue;

                    if (keypoints_temp.size() == 0) {
                        cloud = model_cloud;
                        reference_pose = visy::tools::invertTransformationMatrix(model_pose);
                        view_keypoints_rotated = view_keypoints;
                        descriptor_temp = view_descriptor;
                    } else {
                        Eigen::Matrix4f inv = reference_pose*model_pose;
                        //          view_keypoints_rotated = view_keypoints;
                        visy::extractors::KeyPoint3D::transformKeyPoint3Ds(view_keypoints, view_keypoints_rotated, inv);
                        cv::vconcat(descriptor_temp, view_descriptor, descriptor_temp);
                        pcl::transformPointCloud(*model_cloud, *model_cloud, inv);
                        cloud->points.insert(cloud->points.end(), model_cloud->points.begin(), model_cloud->points.end());
                    }

                    keypoints_temp.insert(keypoints_temp.end(), view_keypoints_rotated.begin(), view_keypoints_rotated.end());

                }

                detector->refineKeyPoints3D(keypoints_temp, descriptor_temp, keypoints, descriptor);
            } else {
                std::vector<visy::extractors::KeyPoint3D> keypoints_temp;
                std::vector<visy::extractors::KeyPoint3D> keypoints_filtered;
                fetchFullModelSimple(model_name, views_max_number, keypoints_temp, cloud, reference_pose, detector);
                cv::Mat fake;
                detector->refineKeyPoints3D(keypoints_temp, keypoints_filtered);
                detector->descriptor->describe(fake, cloud, keypoints_filtered, descriptor);
                visy::extractors::utils::replicateKeypoints(keypoints_filtered, keypoints, detector->getMultiplicationFactor());
            }


        }

        std::string WillowDataset::patternNumberSubstitution(std::string pattern, std::string patternChar, int number) {
            std::string ns = pattern;
            size_t padding = std::count(ns.begin(), ns.end(), patternChar.at(0));
            std::stringstream ss;
            for (int i = 0; i < padding; i++) {
                ss << patternChar;
            }
            std::string pattern_string = ss.str();
            int pattern_index = 0;
            pattern_index = ns.find(pattern_string, pattern_index);
            ns.replace(pattern_index, padding, paddedNumber(number, padding));
            return ns;
        }

        void WillowDataset::loadScene(int set_number, int scene_number, pcl::PointCloud<PointType>::Ptr scene, cv::Mat& rgb_scene) {


            std::stringstream path;
            path << BASE_PATH << SETS_PATH << patternNumberSubstitution(SET_PATTERN_NAME, "$", set_number) << "/";

            std::stringstream cloud_filename;
            cloud_filename << path.str() << patternNumberSubstitution(SCENE_PATTERN_NAME, "$", scene_number) << ".pcd";

            std::cout << "Loading cloud: " << cloud_filename.str() << std::endl;

            if (pcl::io::loadPCDFile(cloud_filename.str(), *scene) < 0) {
                std::cout << "Error loading cloud." << std::endl;
            }

            visy::tools::rgbFromCloud(scene, rgb_scene);
        }

        int WillowDataset::checkInstancesNumber(std::string model_name, std::vector<Annotation>& annotations) {
            int counter = 0;
            for (int i = 0; i < annotations.size(); i++) {
                if (annotations[i].model_name == model_name){
                    if(annotations[i].occlusion <= 0.9001f){
                        counter++;
                    }
                }
            }
            return counter;
        }

        void WillowDataset::loadFullAnnotiationsFromSceneFile(int set_number, int scene_number, std::vector<Annotation>& annotations) {
            annotations.clear();
            for (int i = 0; i < this->models->size(); i++) {
                std::vector<Annotation> instances;
                loadAnnotiationsFromSceneFile((*this->models)[i].name, set_number, scene_number, instances);
                annotations.insert(annotations.end(), instances.begin(), instances.end());
            }
        }

        void WillowDataset::loadAnnotiationsFromSceneFile(std::string model_name, int set_number, int scene_number, std::vector<Annotation>& annotations) {
            annotations.clear();

            std::stringstream ss;
            ss << BASE_PATH << ANNOTATION_PATH;
            ss << patternNumberSubstitution(SET_PATTERN_NAME, "$", set_number);
            ss << "/";
            ss << patternNumberSubstitution(SCENE_PATTERN_NAME, "$", scene_number) << "_";
            std::string base = ss.str();

            int index = 0;
            for (;;) {
                std::string pose_file_name;
                pose_file_name.append(base);
                pose_file_name.append(model_name);
                pose_file_name.append("_");
                pose_file_name.append(std::to_string(index));
                pose_file_name.append(".txt");

                std::string occlusion_file_name;
                occlusion_file_name.append(base);
                occlusion_file_name.append("occlusion_");
                occlusion_file_name.append(model_name);
                occlusion_file_name.append("_");
                occlusion_file_name.append(std::to_string(index));
                occlusion_file_name.append(".txt");

                std::ifstream pose_file(pose_file_name);
                std::ifstream occlusion_file(occlusion_file_name);

                if (pose_file.is_open()) {
                    Annotation annotation;
                    annotation.model_name = model_name;
                    annotation.set_index = set_number;
                    annotation.scene_index = scene_number;

                    for (int i = 0; i < 4; i++) {
                        for (int j = 0; j < 4; j++) {
                            pose_file >> annotation.pose(i, j);
                        }
                    }
                    occlusion_file >> annotation.occlusion;
                    annotations.push_back(annotation);
                    index++;
                } else {
                    break;
                }
            }
        }




    }
}