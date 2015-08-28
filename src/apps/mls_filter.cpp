#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/surface/mls.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "Parameters.h"
#include "Voxy.h"

typedef pcl::PointNormal PointNormalType;
typedef pcl::PointNormal XYZNormalType;

void computeNormals(pcl::PointCloud<PointType>::Ptr& cloud, pcl::PointCloud<NormalType>::Ptr& cloud_normals) {
    pcl::NormalEstimationOMP<PointType, NormalType> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType> ());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*cloud_normals);
}


visy::Parameters * parameters;
pcl::visualization::PCLVisualizer* viewer;

void showCloud(pcl::PointCloud<PointType>::Ptr cloud, double r, double g, double b, float size, std::string name) {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(cloud, r, g, b);
    viewer->addPointCloud<PointType> (cloud, single_color, name.c_str());
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name.c_str());
}

void filterCloudMLS(pcl::PointCloud<PointType>::Ptr& cloud_in, pcl::PointCloud<PointType>::Ptr& cloud_out) {
    // Create a KD-Tree
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<XYZNormalType>::Ptr mls_points(new pcl::PointCloud<XYZNormalType>());
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquaresOMP<PointType, XYZNormalType> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud_in);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(parameters->getFloat("radius"));
    mls.setPolynomialOrder(parameters->getFloat("polinomial"));
    //    mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointType,XYZNormalType>::SAMPLE_LOCAL_PLANE);
    //    mls.setUpsamplingRadius(0.005);
    //    mls.setUpsamplingStepSize(0.003);
    // Reconstruct
    mls.process(*mls_points);


    for (int i = 0; i < mls_points->points.size(); i++) {
        PointType p;
        pcl::PointNormal n = mls_points->points[i];
        p.x = n.x;
        p.y = n.y;
        p.z = n.z;
        p.r = 255;
        p.g = 255;
        p.b = 255;

        cloud_out->points.push_back(p);
    }
}

int
main(int argc, char** argv) {

    /** PARAMETERS */
    parameters = new visy::Parameters(argc, argv);
    parameters->putString("source");
    parameters->putFloat("leaf");
    parameters->putFloat("radius");
    parameters->putFloat("polinomial");

    /* VIEWER */
    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");
    //    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);
    int v0, v1;
    viewer->createViewPort(0, 0, 0.5, 1, v0);
    viewer->createViewPort(0.5, 0, 1, 1, v1);

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_smooth(new pcl::PointCloud<PointType>());


    std::string file = parameters->getString("source");
    if (pcl::io::loadPCDFile<PointType> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    }



    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud);
    float leaf_size = parameters->getFloat("leaf");
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_filtered);

    filterCloudMLS(cloud_filtered, cloud_smooth);

    viewer->addPointCloud(cloud, "cloud", v0);
    viewer->addPointCloud(cloud_smooth, "cloud_filtered", v1);


    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }


    return (0);
}