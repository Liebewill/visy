/* 
 * File:   Dataset.cpp
 * Author: daniele
 * 
 * Created on 9 marzo 2015, 17.51
 */

#include "Dataset.h"
namespace visy
{
  namespace dataset
  {

    std::string Dataset::DATASET_BASE_FOLDER = "datasets";

    Dataset::Dataset ()
    {
    }

    Dataset::~Dataset ()
    {
    }

    void
    Dataset::storeKeyPoint3D (visy::extractors::KeyPoint3D& kp3d, std::string name, cv::FileStorage& storage)
    {
      std::vector<float> v;

      v.push_back(kp3d.pt.x);
      v.push_back(kp3d.pt.y);
      v.push_back(kp3d.pt1.x);
      v.push_back(kp3d.pt1.y);
      v.push_back(kp3d.pt2.x);
      v.push_back(kp3d.pt2.y);
      v.push_back(kp3d.pt3D.x);
      v.push_back(kp3d.pt3D.y);
      v.push_back(kp3d.pt3D.z);
      v.push_back(kp3d.pt3D_1.x);
      v.push_back(kp3d.pt3D_1.y);
      v.push_back(kp3d.pt3D_1.z);
      v.push_back(kp3d.pt3D_2.x);
      v.push_back(kp3d.pt3D_2.y);
      v.push_back(kp3d.pt3D_2.z);
      v.push_back(kp3d.direction_x.x);
      v.push_back(kp3d.direction_x.y);
      v.push_back(kp3d.direction_x.z);
      v.push_back(kp3d.direction_y.x);
      v.push_back(kp3d.direction_y.y);
      v.push_back(kp3d.direction_y.z);
      v.push_back(kp3d.direction_z.x);
      v.push_back(kp3d.direction_z.y);
      v.push_back(kp3d.direction_z.z);
      v.push_back(kp3d.type);
      v.push_back(kp3d.angle);
      v.push_back(kp3d.size);

      storage << name << v;
    }

    void
    Dataset::loadKeyPoint3D (visy::extractors::KeyPoint3D& kp3d, std::string name, cv::FileStorage& storage)
    {
      std::vector<float> v;
      storage[name] >> v;

      int index = 0;
      kp3d.pt.x = v[index++];
      kp3d.pt.y = v[index++];
      kp3d.pt1.x = v[index++];
      kp3d.pt1.y = v[index++];
      kp3d.pt2.x = v[index++];
      kp3d.pt2.y = v[index++];
      kp3d.pt3D.x = v[index++];
      kp3d.pt3D.y = v[index++];
      kp3d.pt3D.z = v[index++];
      kp3d.pt3D_1.x = v[index++];
      kp3d.pt3D_1.y = v[index++];
      kp3d.pt3D_1.z = v[index++];
      kp3d.pt3D_2.x = v[index++];
      kp3d.pt3D_2.y = v[index++];
      kp3d.pt3D_2.z = v[index++];
      kp3d.direction_x.x = v[index++];
      kp3d.direction_x.y = v[index++];
      kp3d.direction_x.z = v[index++];
      kp3d.direction_y.x = v[index++];
      kp3d.direction_y.y = v[index++];
      kp3d.direction_y.z = v[index++];
      kp3d.direction_z.x = v[index++];
      kp3d.direction_z.y = v[index++];
      kp3d.direction_z.z = v[index++];
      kp3d.type = v[index++];
      kp3d.angle = v[index++];
      kp3d.size = v[index++];

    }

    void
    Dataset::storeM44 (Eigen::Matrix4f& matrix, std::string name, cv::FileStorage& storage)
    {
      std::vector<float> temp(16);
      for (int z = 0; z < 16; z++)
      {
        temp[z] = matrix(z / 4, z % 4);
      }
      storage << name << temp;
    }

    void
    Dataset::loadM44 (Eigen::Matrix4f& matrix, std::string name, cv::FileStorage& storage)
    {
      std::vector<float> temp;
      storage[name.c_str()] >> temp;
      for (int z = 0; z < 16; z++)
      {
        matrix(z / 4, z % 4) = temp[z];
      }
    }

    void
    Dataset::saveDescription (std::string model_name, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, Eigen::Matrix4f& reference_pose, std::string description_name)
    {
      this->createBaseFolder();
      this->createDatasetFolder();
      this->createModelFolder(model_name);

      std::string file_name = this->getFileOfModelPath(model_name, description_name, "description");
      cv::FileStorage fs(file_name, cv::FileStorage::WRITE);

      float size = keypoints.size();
      fs << "size" << size;

      std::stringstream ss;
      for (int i = 0; i < keypoints.size(); i++)
      {
        ss.str("");
        ss << "kp_" << i;
        storeKeyPoint3D(keypoints[i], ss.str(), fs);
      }
      fs << "descriptor" << descriptor;
      std::cout << "Saved size: " << keypoints.size() << "/" << descriptor.rows << "x" << descriptor.cols << std::endl;
      storeM44(reference_pose, "reference_pose", fs);

      fs.release();
    }

    bool
    Dataset::loadDescription (std::string model_name, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, Eigen::Matrix4f& reference_pose, std::string description_name)
    {
      this->createBaseFolder();
      this->createDatasetFolder();
      this->createModelFolder(model_name);

      std::string file_name = this->getFileOfModelPath(model_name, description_name, "description");
      cv::FileStorage fs(file_name, cv::FileStorage::READ);
      if (!fs.isOpened())
      {
        return false;
      }

      float size = -1.0f;
      fs["size"] >> size;


      std::stringstream ss;
      for (int i = 0; i < size; i++)
      {
        ss.str("");
        ss << "kp_" << i;
        visy::extractors::KeyPoint3D kp3d;
        loadKeyPoint3D(kp3d, ss.str(), fs);
        keypoints.push_back(kp3d);
      }
      fs["descriptor"] >> descriptor;
      loadM44(reference_pose, "reference_pose", fs);

      std::cout << "Loaded size: " << keypoints.size() << "/" << descriptor.rows << "x" << descriptor.cols << std::endl;
      fs.release();
    }

    void
    Dataset::savePrecisionMat (std::string descriptor_name, cv::Mat& precisionMat)
    {
      this->createBaseFolder();
      this->createDatasetFolder();
      this->createPrecisionFolder();

      std::string file_name = getFilePrecisionPath(descriptor_name, "data");
      std::string file_name_csv = getFilePrecisionPath(descriptor_name, "csv");

      std::cout << "PRECISON FILE: " << file_name << std::endl;
      cv::Mat previousMat(precisionMat.rows, precisionMat.cols, precisionMat.type(),float(0));

      cv::FileStorage fs(file_name, cv::FileStorage::READ);
      if (fs.isOpened())
      {
        fs["precision"] >> previousMat;
        fs.release();
      }

      cv::Mat outMat = precisionMat + previousMat;


      cv::FileStorage fw(file_name, cv::FileStorage::WRITE);
      fw << "precision" << outMat;
      fw.release();

      ofstream csv;
      csv.open(file_name_csv.c_str());
      csv << std::fixed;
      for (int i = 0; i < outMat.rows; i++)
      {
        for (int j = 0; j < outMat.cols; j++)
        {
          csv << outMat.at<float>(i, j);
          if (j < outMat.cols - 1)
          {
            csv << ";";
          }

        }
        csv << "\n";
      }
      csv.close();
    }

    void
    Dataset::createBaseFolder ()
    {
      boost::filesystem::create_directories(DATASET_BASE_FOLDER);
    }

    std::string
    Dataset::getDatasetFolder ()
    {
      std::stringstream ss;
      ss << DATASET_BASE_FOLDER << "/" << this->name << "/";
      return ss.str();
    }

    std::string
    Dataset::getModelFolder (std::string model_name)
    {
      std::stringstream ss;
      ss << this->getDatasetFolder() << model_name << "/";
      return ss.str();
    }

    std::string
    Dataset::getFileOfModelPath (std::string model_name, std::string file_name, std::string extension)
    {
      std::stringstream ss;
      ss << this->getModelFolder(model_name) << file_name << "." << extension;
      return ss.str();
    }

    std::string
    Dataset::getFilePrecisionPath (std::string file_name, std::string extension)
    {
      std::stringstream ss;
      ss << this->getPrecisionFolder() << file_name << "." << extension;
      return ss.str();
    }

    std::string
    Dataset::getPrecisionFolder ()
    {
      std::stringstream ss;
      ss << this->getDatasetFolder() << "precision/";
      return ss.str();
    }

    void
    Dataset::createPrecisionFolder ()
    {
      boost::filesystem::create_directories(this->getPrecisionFolder());
    }

    void
    Dataset::createModelFolder (std::string model_name)
    {
      boost::filesystem::create_directories(this->getModelFolder(model_name));
    }

    void
    Dataset::createDatasetFolder ()
    {
      boost::filesystem::create_directories(this->getDatasetFolder());
    }


  }
}