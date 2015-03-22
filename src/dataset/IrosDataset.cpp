/* 
 * File:   IrosDataset.cpp
 * Author: daniele
 * 
 * Created on 20 novembre 2014, 23.19
 */

#include <pcl-1.7/pcl/common/io.h>
#include <pcl-1.7/pcl/common/transforms.h>

#include "IrosDataset.h"
#include "Detector.h"
#include "Dataset.h"

namespace visy
{
  namespace dataset
  {
    std::string IrosDataset::BASE_PATH = "/home/daniele/iros_dataset/";
    std::string IrosDataset::TRAINING_PATH = "training_data/";
    std::string IrosDataset::SETS_PATH = "test_set/";
    std::string IrosDataset::ANNOTATION_PATH = "annotations/";
    std::vector<Model>* IrosDataset::models = new std::vector<Model>();
    std::vector<SetScene>* IrosDataset::scenes = new std::vector<SetScene>();

    IrosDataset::IrosDataset () : Dataset ()
    {
      this->name = "IROS-TW";
    }

    IrosDataset::~IrosDataset ()
    {
    }

    void
    IrosDataset::init ()
    {
      IrosDataset::models->push_back(Model("asus_box", 37));
      IrosDataset::models->push_back(Model("burti", 60));
      IrosDataset::models->push_back(Model("canon_camera_bag", 38));
      IrosDataset::models->push_back(Model("cisco_phone", 19));
      IrosDataset::models->push_back(Model("coffee_container", 16));
      IrosDataset::models->push_back(Model("felix_ketchup", 20));
      IrosDataset::models->push_back(Model("fruchtmolke", 49));
      IrosDataset::models->push_back(Model("jasmine_green_tea", 41));
      IrosDataset::models->push_back(Model("muller_milch_banana", 20));
      IrosDataset::models->push_back(Model("muller_milch_shoko", 19));
      IrosDataset::models->push_back(Model("opencv_book", 12));
      IrosDataset::models->push_back(Model("red_mug_white_spots", 79));
      IrosDataset::models->push_back(Model("skull", 39));
      IrosDataset::models->push_back(Model("strands_mounting_unit", 56));
      IrosDataset::models->push_back(Model("toilet_paper", 16));
      IrosDataset::models->push_back(Model("water_boiler", 19));
      IrosDataset::models->push_back(Model("yellow_toy_car", 19));

      //SCENES
      IrosDataset::scenes->push_back(SetScene(1, 6));
      IrosDataset::scenes->push_back(SetScene(2, 6));
      IrosDataset::scenes->push_back(SetScene(3, 13));
      IrosDataset::scenes->push_back(SetScene(4, 15));
      IrosDataset::scenes->push_back(SetScene(5, 9));
      IrosDataset::scenes->push_back(SetScene(6, 12));
      IrosDataset::scenes->push_back(SetScene(7, 7));
      IrosDataset::scenes->push_back(SetScene(8, 7));
      IrosDataset::scenes->push_back(SetScene(9, 8));
      IrosDataset::scenes->push_back(SetScene(10, 6));
      IrosDataset::scenes->push_back(SetScene(11, 8));
      IrosDataset::scenes->push_back(SetScene(12, 7));
      IrosDataset::scenes->push_back(SetScene(13, 17));
      IrosDataset::scenes->push_back(SetScene(14, 15));
      IrosDataset::scenes->push_back(SetScene(15, 12));
    }

    Model
    IrosDataset::findModelByName (std::string simpleName)
    {
      for (int i = 0; i < IrosDataset::models->size(); i++)
      {
        if (IrosDataset::models->at(i).name == simpleName)
        {
          return IrosDataset::models->at(i);
        }
      }
      return Model("INVALID", 0);
    }

    void
    IrosDataset::loadModel (std::string simpleName, int view_number, pcl::PointCloud<PointType>::Ptr full_model, pcl::PointCloud<PointType>::Ptr model, cv::Mat& rgb_model, Eigen::Matrix4f& pose)
    {

      std::stringstream path;
      path << BASE_PATH << TRAINING_PATH << simpleName << ".pcd/";

      std::stringstream cloud_filename;
      cloud_filename << path.str() << "cloud_" << paddedNumber(view_number, 8) << ".pcd";
      std::stringstream indices_filename;
      indices_filename << path.str() << "object_indices_" << paddedNumber(view_number, 8) << ".pcd.ply";

      if (pcl::io::loadPCDFile(cloud_filename.str(), *full_model) < 0)
      {
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

    void
    IrosDataset::loadScene (int set_number, int scene_number, pcl::PointCloud<PointType>::Ptr scene, cv::Mat& rgb_scene)
    {
      std::stringstream path;
      path << BASE_PATH << SETS_PATH << "set_" << paddedNumber(set_number, 5) << "/";

      std::stringstream cloud_filename;
      cloud_filename << path.str() << paddedNumber(scene_number, 5) << ".pcd";

      std::cout << "Loading cloud: " << cloud_filename.str() << std::endl;

      if (pcl::io::loadPCDFile(cloud_filename.str(), *scene) < 0)
      {
        std::cout << "Error loading cloud." << std::endl;
      }

      visy::tools::rgbFromCloud(scene, rgb_scene);
    }

    void
    IrosDataset::loadModel (std::string simpleName, int view_number, pcl::PointCloud<PointType>::Ptr full_model, pcl::PointCloud<PointType>::Ptr model, cv::Mat& rgb_model, cv::Mat& rgb_full_model, Eigen::Matrix4f& pose)
    {
      loadModel(simpleName, view_number, full_model, model, rgb_model, pose);
      visy::tools::rgbFromCloud(full_model, rgb_full_model);
    }

    /**
     * Foolish method
     * @param path
     * @param indices
     */
    void
    IrosDataset::loadIndices (std::string path, std::vector<int>& indices)
    {
      std::ifstream infile(path.c_str());
      std::string line;
      std::string key;
      int count = -1;
      int i = 0;
      bool is_data = false;
      int* idxs;
      while (std::getline(infile, line))
      {
        std::istringstream iss(line);

        if (is_data)
        {
          iss >> idxs[i];
          indices.push_back(idxs[i]);
          i++;
          if (i == count - 1)
          {
            break;
          }
        }
        else
        {
          iss>>key;

          if (key == "element")
          {
            iss>>key;
            if (key == "vertex")
            {
              iss>>count;
              idxs = new int[count];
              //std::cout << "Reading size:" << count << std::endl;
            }
          }

          if (key == "end_header")
          {
            is_data = true;
          }
        }
      }
      delete[] idxs;


    }

    std::string
    IrosDataset::paddedNumber (int number, int length, bool after)
    {
      std::stringstream id;
      id << number;
      int pad = length;
      pad = pad - id.str().length();
      id.str("");
      for (int i = 0; i < pad; i++)
      {
        id << "0";
      }
      id << number;
      return id.str();
    }

    void
    IrosDataset::loadPoseFromFile (std::string path, Eigen::Matrix4f& pose)
    {
      std::ifstream infile(path.c_str());

      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          infile >> pose(i, j);
        }
      }

    }

    void
    IrosDataset::loadPoseFromSceneFile (std::string model_name, int set_number, int scene_number, int model_index, Eigen::Matrix4f& pose)
    {
      std::stringstream ss;
      ss << BASE_PATH << ANNOTATION_PATH << "set_" << paddedNumber(set_number, 5) << "/";
      ss << paddedNumber(scene_number, 5) << "_" << model_name << "_" << model_index << ".txt";
      std::ifstream infile(ss.str().c_str());
      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          infile >> pose(i, j);
        }
      }
    }

    void
    IrosDataset::loadAnnotiationsFromSceneFile (std::string model_name, int set_number, int scene_number, std::vector<Annotation>& annotations)
    {
      annotations.clear();

      std::stringstream ss;
      ss << BASE_PATH << ANNOTATION_PATH << "set_" << paddedNumber(set_number, 5) << "/" << paddedNumber(scene_number, 5) << "_";
      std::string base = ss.str();

      int index = 0;
      for (;;)
      {
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

        if (pose_file.is_open())
        {
          Annotation annotation;
          annotation.model_name = model_name;
          annotation.set_index = set_number;
          annotation.scene_index = scene_number;

          for (int i = 0; i < 4; i++)
          {
            for (int j = 0; j < 4; j++)
            {
              pose_file >> annotation.pose(i, j);
            }
          }
          occlusion_file >> annotation.occlusion;
          annotations.push_back(annotation);
          index++;
        }
        else
        {
          break;
        }
      }


    }

    void
    IrosDataset::loadFullAnnotiationsFromSceneFile (int set_number, int scene_number, std::vector<Annotation>& annotations)
    {
      annotations.clear();
      for (int i = 0; i < IrosDataset::models->size(); i++)
      {
        std::vector<Annotation> instances;
        IrosDataset::loadAnnotiationsFromSceneFile((*IrosDataset::models)[i].name, set_number, scene_number, instances);
        annotations.insert(annotations.end(), instances.begin(), instances.end());
      }
    }

    int
    IrosDataset::checkInstancesNumber (std::string model_name, std::vector<Annotation>& annotations)
    {
      int counter = 0;
      for (int i = 0; i < annotations.size(); i++)
      {
        if (annotations[i].model_name == model_name)
          counter++;
      }
      return counter;
    }

    bool
    IrosDataset::checkHV (std::string model_name, Eigen::Matrix4f pose, std::vector<Annotation>& annotations, float distance_th, float rot_th)
    {
      for (int i = 0; i < annotations.size(); i++)
      {
        if (annotations[i].model_name == model_name)
        {
          float rot_error, distance_error;
          visy::tools::poseError(pose, annotations[i].pose, rot_error, distance_error);
          if (distance_error < distance_th && rot_error < rot_th)
            return true;
        }
      }
      return false;
    }

    bool
    IrosDataset::checkPoses (std::string model_name, std::vector<Eigen::Matrix4f>& poses, std::vector<Annotation>& annotations)
    {
      for (int i = 0; i < annotations.size(); i++)
      {
        if (annotations[i].model_name == model_name)
        {
          poses.push_back(annotations[i].pose);
        }
      }
    }

    void
    IrosDataset::fetchFullModel (std::string model_name, int views_max_number, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, pcl::PointCloud<PointType>::Ptr& cloud, Eigen::Matrix4f& reference_pose, visy::detectors::Detector* detector)
    {
      if (detector->isDetectorEmbedded())
      {
        std::vector<visy::extractors::KeyPoint3D> keypoints_temp;
        cv::Mat descriptor_temp;

        keypoints_temp.clear();

        for (int i = 0; i <= views_max_number; i++)
        {
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

          if (keypoints_temp.size() == 0)
          {
            cloud = model_cloud;
            reference_pose = visy::tools::invertTransformationMatrix(model_pose);
            view_keypoints_rotated = view_keypoints;
            descriptor_temp = view_descriptor;
          }
          else
          {
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
      }
      else
      { 
        std::vector<visy::extractors::KeyPoint3D> keypoints_temp;
        std::vector<visy::extractors::KeyPoint3D> keypoints_filtered;
        fetchFullModelSimple(model_name, views_max_number, keypoints_temp, cloud, reference_pose, detector);
        cv::Mat fake;
        detector->refineKeyPoints3D(keypoints_temp,keypoints_filtered);
        detector->descriptor->describe(fake, cloud, keypoints_filtered, descriptor);
        visy::extractors::utils::replicateKeypoints(keypoints_filtered, keypoints, detector->getMultiplicationFactor());
      }
    }

    void
    IrosDataset::fetchFullModelSimple (std::string model_name, int views_max_number, std::vector<visy::extractors::KeyPoint3D>& keypoints, pcl::PointCloud<PointType>::Ptr& cloud, Eigen::Matrix4f& reference_pose, visy::detectors::Detector* detector)
    {
      std::vector<visy::extractors::KeyPoint3D> keypoints_temp;

      keypoints_temp.clear();

      for (int i = 0; i <= views_max_number; i++)
      {
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

        if (keypoints_temp.size() == 0)
        {
          cloud = model_cloud;
          reference_pose = visy::tools::invertTransformationMatrix(model_pose);
          view_keypoints_rotated = view_keypoints;

        }
        else
        {
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



  }
}