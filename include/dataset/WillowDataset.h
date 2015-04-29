/* 
 * File:   WillowDataset.h
 * Author: daniele
 *
 * Created on April 22, 2015, 10:16 AM
 */

#ifndef WILLOWDATASET_H
#define	WILLOWDATASET_H

#include "tools.h"
#include "detectors/Detector.h"
#include "Dataset.h"
#include "pcl/common/transforms.h"

namespace visy {
    namespace dataset {

        class WillowDataset : public Dataset {
        public:
            std::string BASE_PATH;
            std::string TRAINING_PATH;
            std::string SETS_PATH;
            std::string SET_PATTERN_NAME;
            std::string SCENE_PATTERN_NAME;
            std::string ANNOTATION_PATH;
            std::vector<Model>* models;
            std::vector<SetScene>* scenes;
            std::string type;

            WillowDataset(std::string name = "WILLOW");
            virtual ~WillowDataset();




            void init();
            void loadIndices(std::string path, std::vector<int>& indices);
            void loadPoseFromFile(std::string path, Eigen::Matrix4f& pose);
            Model findModelByName(std::string simpleName);

            void loadScene(int set_number, int scene_number, pcl::PointCloud<PointType>::Ptr scene, cv::Mat& rgb_scene);


            void loadModel(std::string simpleName, int view_number, pcl::PointCloud<PointType>::Ptr full_model, pcl::PointCloud<PointType>::Ptr model, cv::Mat& rgb_model, cv::Mat& rgb_full_model, Eigen::Matrix4f& pose);
            void fetchFullModel(std::string model_name, int views_max_number, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, pcl::PointCloud<PointType>::Ptr& cloud, Eigen::Matrix4f& reference_pose, visy::detectors::Detector * detector);
            void fetchFullModelSimple(std::string model_name, int views_max_number, std::vector<visy::extractors::KeyPoint3D>& keypoints, pcl::PointCloud<PointType>::Ptr& cloud, Eigen::Matrix4f& reference_pose, visy::detectors::Detector * detector);

            void loadAnnotiationsFromSceneFile(std::string model_name, int set_number, int scene_number, std::vector<Annotation>& annotations);
            void loadFullAnnotiationsFromSceneFile(int set_number, int scene_number, std::vector<Annotation>& annotations);
            int checkInstancesNumber(std::string model_name, std::vector<Annotation>& annotations);

            std::string paddedNumber(int number, int length, bool after = false);
            std::string patternNumberSubstitution(std::string pattern, std::string patternChar, int number);
        private:

        };
    }
}
#endif	/* WILLOWDATASET_H */

