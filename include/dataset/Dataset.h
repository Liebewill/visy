/* 
 * File:   Dataset.h
 * Author: daniele
 *
 * Created on 9 marzo 2015, 17.51
 */

#ifndef DATASET_H
#define	DATASET_H

#include "extractors/KeyPoint3D.h"
#include <boost/filesystem.hpp>

namespace visy {
    namespace dataset {

        class Dataset {
        public:

            static std::string DATASET_BASE_FOLDER;

            Dataset();
            virtual ~Dataset();
            void saveDescription(std::string model_name, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, Eigen::Matrix4f& reference_pose, std::string description_name);
            bool loadDescription(std::string model_name, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, Eigen::Matrix4f& reference_pose, std::string description_name);
            static void storeKeyPoint3D(visy::extractors::KeyPoint3D& kp3d, std::string name, cv::FileStorage& storage);
            static void loadKeyPoint3D(visy::extractors::KeyPoint3D& kp3d, std::string name, cv::FileStorage& storage);
            static void storeM44(Eigen::Matrix4f& matrix, std::string name, cv::FileStorage& storage);
            static void loadM44(Eigen::Matrix4f& matrix, std::string name, cv::FileStorage& storage);
        protected:
            std::string name;
            std::string getDatasetFolder();
            std::string getModelFolder(std::string model_name);
            std::string getFileOfModelPath(std::string model_name, std::string file_name, std::string extension);
            void createModelFolder(std::string model_name);
            void createDatasetFolder();
            void createBaseFolder();

        };
    }
}
#endif	/* DATASET_H */

