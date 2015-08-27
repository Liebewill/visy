#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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

#include <pcl/filters/voxel_grid.h>

#include "Parameters.h"
#include "Voxy.h"

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

int
main(int argc, char** argv) {

    /** PARAMETERS */
    parameters = new visy::Parameters(argc, argv);
    parameters->putString("source");

    /* VIEWER */
    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");
    //    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

    /** ELEVATOR MAP */
    emap = new ElevatorMap(2.0, 0.01, 1.5);

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_planes(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_clusters(new pcl::PointCloud<PointType>());
    pcl::PointCloud<NormalType>::Ptr cloud_normals(new pcl::PointCloud<NormalType>());


    std::string file = parameters->getString("source");
    if (pcl::io::loadPCDFile<PointType> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    }

    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud);
    float leaf_size = 0.005f;
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud);

    
    computeNormals(cloud, cloud_normals);

    std::vector<int> planes_indices;
    std::vector<int> withoutplanes_indices;

    boost::posix_time::ptime time_start, time_end;
    boost::posix_time::time_duration duration;

    time_start = boost::posix_time::microsec_clock::local_time();
    emap->planesCheck(
            cloud,
            cloud_normals,
            withoutplanes_indices,
            planes_indices,
            8.0f,
            6500);
    time_end = boost::posix_time::microsec_clock::local_time();
    duration = time_end - time_start;
    std::cout << "Planes check: " << duration << std::endl;

    std::cout << "Planes size:" << planes_indices.size() << "/" << withoutplanes_indices.size() << std::endl;
    pcl::copyPointCloud(*cloud, planes_indices, *cloud_planes);

    viewer->addPointCloud(cloud, "cloud_source");
    //    showCloud(cloud_planes, 255, 0, 0, 4, "planes");
    
    
     time_start = boost::posix_time::microsec_clock::local_time();


    pcl::PointCloud<PointNormalType>::Ptr cloud_with_normals(new pcl::PointCloud<PointNormalType>);
    for (int i = 0; i < cloud->points.size(); i++) {
        PointNormalType pn;
        PointType p = cloud->points[i];
        NormalType n = cloud_normals->points[i];

        pn.x = p.x;
        pn.y = p.y;
        pn.z = p.z;
        pn.normal_x = n.normal_x;
        pn.normal_y = n.normal_y;
        pn.normal_z = n.normal_z;
        cloud_with_normals->points.push_back(pn);
    }
    std::cout << "Cloud with normals: " << cloud_with_normals->points.size() << std::endl;
    //    pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<PointNormalType>::Ptr tree2(new pcl::search::KdTree<PointNormalType>);
    tree2->setInputCloud(cloud_with_normals);
    
    // Initialize objects
    pcl::GreedyProjectionTriangulation<PointNormalType> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.025);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    
     time_end = boost::posix_time::microsec_clock::local_time();
    duration = time_end - time_start;
    std::cout << "Triangulation: " << duration << std::endl;

    
    viewer->addPolygonMesh(triangles);


    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }


    return (0);
}