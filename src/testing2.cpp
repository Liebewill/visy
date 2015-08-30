#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/transforms.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/filters/fast_bilateral.h>

#include <pcl/filters/voxel_grid.h>

#include "Parameters.h"
#include "Voxy.h"
#include "tools.h"

typedef pcl::PointNormal PointNormalType;

void computeNormals(pcl::PointCloud<PointType>::Ptr& cloud, pcl::PointCloud<NormalType>::Ptr& cloud_normals) {
    pcl::NormalEstimationOMP<PointType, NormalType> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType> ());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*cloud_normals);
}

struct ElevatorMap {
    double* map;
    double size;
    double max_size;
    double step;
    double offset;

    ElevatorMap(double max_size, double step, double offset) {
        this->size = max_size / step;
        this->map = new double[(int) size];
        this->max_size = max_size;
        this->step = step;
        this->offset = offset;
        std::fill(this->map, this->map + (int) this->size, 0.0);
    }

    void pinPoint(double z) {
        int iz = floor(z / step);
        iz += floor(this->offset / step);
        if (iz<this->size && iz >= 0) {
            this->map[iz]++;
        }
    }

    int pointIndex(double z) {
        int iz = floor(z / step);
        iz += floor(this->offset / step);
        if (iz<this->size && iz >= 0) {
            return iz;
        }
        return -1;
    }

    double pointValue(double z) {
        int iz = floor(z / step);
        iz += floor(this->offset / step);
        if (iz<this->size && iz >= 0) {
            return this->map[iz];
        }
        return -1;
    }

    void planesCheck(
            pcl::PointCloud<PointType>::Ptr& cloud,
            pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
            std::vector<int>& filtered_indices,
            std::vector<int>& planes_indices,
            float max_angle,
            int map_min_inliers
            ) {

        Eigen::Vector3f normal;
        Eigen::Vector3f gravity_neg(0, 0, 1);


        for (int i = 0; i < cloud_normals->points.size(); i++) {
            pcl::Normal n = cloud_normals->points[i];
            PointType p = cloud->points[i];
            normal(0) = n.normal_x;
            normal(1) = n.normal_y;
            normal(2) = n.normal_z;

            float angle = acos(normal.dot(gravity_neg));
            if (angle <= max_angle * M_PI / 180.0f) {
                this->pinPoint(p.z);
            }
        }

        for (int i = 0; i < cloud->points.size(); i++) {
            PointType p = cloud->points[i];

            if (this->pointValue(p.z) >= map_min_inliers) {
                planes_indices.push_back(i);
            } else if (this->pointValue(p.z - step) >= map_min_inliers) {
                planes_indices.push_back(i);
            } else if (this->pointValue(p.z + step) >= map_min_inliers) {
                planes_indices.push_back(i);
            } else {
                filtered_indices.push_back(i);
            }

        }
    }
};


visy::Parameters * parameters;
pcl::visualization::PCLVisualizer* viewer;
ElevatorMap* emap;

void showCloud(pcl::PointCloud<PointType>::Ptr cloud, double r, double g, double b, float size, std::string name) {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(cloud, r, g, b);
    viewer->addPointCloud<PointType> (cloud, single_color, name.c_str());
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name.c_str());
}

std::string cloud_base_path = "/home/daniele/Desktop/TSDF_Dataset/House_Refined/";

void loadCloud(int index, pcl::PointCloud<PointType>::Ptr& cloud, Eigen::Matrix4f& t) {

    std::stringstream ss;
    ss << cloud_base_path;
    ss << index << ".pcd";
    if (pcl::io::loadPCDFile<PointType> (ss.str().c_str(), *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");

    }

    std::cout << "Points Loaded: " << cloud->points.size() << std::endl;
    //LOAD MATRIX
    ss.str("");
    ss << cloud_base_path;
    ss << index << ".txt";
    ifstream myReadFile;
    std::cout << "Opening: " << ss.str().c_str() << std::endl;
    myReadFile.open(ss.str().c_str());
    char output[100];
    if (myReadFile.is_open()) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                myReadFile >> output;
                t(i, j) = atof(output);
            }
        }
    } else {
        std::cout << "BOH" << std::endl;
    }
    myReadFile.close();

}

int
main2(int argc, char** argv) {

    /** PARAMETERS */
    parameters = new visy::Parameters(argc, argv);
    parameters->putInt("s1");
    parameters->putInt("s2");

    /* VIEWER */
    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");
    //    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

    /** ELEVATOR MAP */
    emap = new ElevatorMap(2.0, 0.01, 1.5);

    pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud2(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud1_t(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud2_t(new pcl::PointCloud<PointType>());

    Eigen::Matrix4f t1;
    Eigen::Matrix4f t2;

    loadCloud(parameters->getInt("s1"), cloud1, t1);
    loadCloud(parameters->getInt("s2"), cloud2, t2);

    pcl::transformPointCloud(*cloud1, *cloud1_t, t1);
    pcl::transformPointCloud(*cloud2, *cloud2_t, t2);

    viewer->addPointCloud(cloud1_t,"c1");
    viewer->addPointCloud(cloud2_t,"c2");


    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }


    return (0);
}
int
main(int argc, char** argv) {

    /** PARAMETERS */
    parameters = new visy::Parameters(argc, argv);
    parameters->putInt("s1");
    parameters->putInt("s2");
    parameters->putFloat("max",60);

    
    
    /* VIEWER */
//    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");
    //    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

    /** ELEVATOR MAP */
    emap = new ElevatorMap(2.0, 0.01, 1.5);

    pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>());
    

    Eigen::Matrix4f t1;
    Eigen::Matrix4f t2;

    for(int i = 0; i <= parameters->getFloat("max");i++){
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        loadCloud(i, cloud, t1);
        cv::Mat img;
        visy::tools::rgbFromCloud(cloud,img);
        
        std::stringstream ss;
        ss << cloud_base_path << i<<".png";
        cv::imwrite(ss.str().c_str(),img);
    }
    


    return (0);
}