#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/bilateral.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>

#include "Parameters.h"
#include "WillowDataset.h"
#include "Voxy.h"



visy::Parameters * parameters;
Eigen::Matrix4f adjust;
int current_index = 0;
visy::Voxy* voxy;
pcl::visualization::PCLVisualizer* viewer;
pcl::PointCloud<PointType>::Ptr full_cloud(new pcl::PointCloud<PointType>());
typedef pcl::PointNormal XYZNormalType;

void loadCloud(int index, pcl::PointCloud<PointType>::Ptr& cloud, Eigen::Matrix4f& t) {

    std::stringstream ss;
    ss << "/home/daniele/Desktop/TSDF_Test/1437572674.743236835/";
    ss << index << ".pcd";
    if (pcl::io::loadPCDFile<PointType> (ss.str().c_str(), *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");

    }

    //LOAD MATRIX
    ss.str("");
    ss << "/home/daniele/Desktop/TSDF_Test/1437572674.743236835/";
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

void addPointCloudToVox(visy::Voxy* voxy, int index) {

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>());
    Eigen::Matrix4f t;
    loadCloud(index, cloud, t);

    t = t*adjust;

    pcl::transformPointCloud(*cloud, *cloud_trans, t);
    Eigen::Vector3f pov(t(0, 3), t(1, 3), t(2, 3));
    voxy->addPointCloud(cloud_trans, pov);

    pcl::PointCloud<PointType>::Ptr cloud_vox(new pcl::PointCloud<PointType>());
    voxy->voxelToCloudZeroCrossing(cloud_vox);




    // Create a KD-Tree
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<XYZNormalType>::Ptr mls_points(new pcl::PointCloud<XYZNormalType>());
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointType, XYZNormalType> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud_vox);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.05);
    mls.setPolynomialOrder(2);
    //    mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointType,XYZNormalType>::SAMPLE_LOCAL_PLANE);
    //    mls.setUpsamplingRadius(0.005);
    //    mls.setUpsamplingStepSize(0.003);
    // Reconstruct
    mls.process(*mls_points);

    pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());

    for (int i = 0; i < mls_points->points.size(); i++) {
        PointType p;
        pcl::PointNormal n = mls_points->points[i];
        p.x = n.x;
        p.y = n.y;
        p.z = n.z;
        p.r = 255;
        p.g = 255;
        p.b = 255;

        scene->points.push_back(p);
    }


    viewer->removeAllPointClouds();
    viewer->addPointCloud(scene, "cloud_vox");
    viewer->spinOnce();

    //    if (current_index > 5) {
    //        addPointCloudToVox(voxy, current_index);
    //        current_index += 5;
    //    }
}

void addPointCloudToViewer(int index) {

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>());
    Eigen::Matrix4f t;
    loadCloud(index, cloud, t);

    t = t*adjust;

    pcl::transformPointCloud(*cloud, *cloud_trans, t);
    Eigen::Vector3f pov(t(0, 3), t(1, 3), t(2, 3));

    (*full_cloud) += (*cloud_trans);


    //    typedef pcl::PointNormal XYZNormalType;
    //
    //    // Create a KD-Tree
    //    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    //
    //    // Output has the PointNormal type in order to store the normals calculated by MLS
    //    pcl::PointCloud<XYZNormalType>::Ptr mls_points(new pcl::PointCloud<XYZNormalType>());
    //    // Init object (second point type is for the normals, even if unused)
    //    pcl::MovingLeastSquares<PointType, XYZNormalType> mls;
    //
    //    mls.setComputeNormals(true);
    //
    //    // Set parameters
    //    mls.setInputCloud(full_cloud);
    //    mls.setPolynomialFit(true);
    //    mls.setSearchMethod(tree);
    //    mls.setSearchRadius(0.05);
    //    mls.setPolynomialOrder(2);
    //    //    mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointType,XYZNormalType>::SAMPLE_LOCAL_PLANE);
    //    //    mls.setUpsamplingRadius(0.005);
    //    //    mls.setUpsamplingStepSize(0.003);
    //    // Reconstruct
    //    mls.process(*mls_points);
    //
    //    pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
    //    
    //    for(int i = 0; i < mls_points->points.size(); i++){
    //        PointType p;
    //        pcl::PointNormal n =mls_points->points[i];
    //        p.x = n.x;
    //        p.y = n.y;
    //        p.z = n.z;
    //        p.r = 255;
    //        p.g = 255;
    //        p.b = 255;
    //        
    //        scene->points.push_back(p);
    //    }

    viewer->removeAllPointClouds();
    viewer->addPointCloud(full_cloud, "cloud_full");
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
        void* viewer_void) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.getKeySym() == "v" && event.keyDown()) {
        //addPointCloudToVox(voxy,current_index);
        if (current_index < 60) {
            //            addPointCloudToViewer(current_index);
            addPointCloudToVox(voxy, current_index);
            current_index += 1;
        }
    }
}

int
main(int argc, char** argv) {
    /** PARAMETERS */
    parameters = new visy::Parameters(argc, argv);


    adjust <<
            1, 0, 0, -0.01,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

    int size = 256;
    int full_size = size * size*size;
    double step = 2.0f / (double) size;

    Eigen::Vector3f offset(0.0, 1.0, 1.0);

    voxy = new visy::Voxy(size, 2.0, 0.01f, offset);


    //    for (int i = 0; i < 50; i+=1) {
    int index = 0;
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>());
    Eigen::Matrix4f t;
    loadCloud(0, cloud, t);
    t = t*adjust;
    pcl::transformPointCloud(*cloud, *cloud_trans, t);
    Eigen::Vector3f pov(t(0, 3), t(1, 3), t(2, 3));

    //    voxy->addPointCloud(cloud_trans, pov);


    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
    std::vector<int> filtered_indices;

    pcl::NormalEstimation<PointType, pcl::Normal> ne;
    ne.setInputCloud(cloud_trans);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType> ());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*cloud_normals);

    Eigen::Vector3f normal;
    Eigen::Vector3f gravity_neg(0, 0, 1);
    for (int i = 0; i < cloud_normals->points.size(); i++) {
        pcl::Normal n = cloud_normals->points[i];
        normal(0) = n.normal_x;
        normal(1) = n.normal_y;
        normal(2) = n.normal_z;

        float angle = acos(normal.dot(gravity_neg));
        if (angle > 10.0f * M_PI / 180.0f) {
            std::cout << angle << std::endl;
            filtered_indices.push_back(i);
        }
    }

    pcl::copyPointCloud(*cloud_trans, filtered_indices, *cloud_filtered);


    pcl::search::KdTree<PointType>::Ptr tree2(new pcl::search::KdTree<PointType>);
    tree2->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree2);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    for (int i = 0; i < cluster_indices.size(); i++) {
        pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>);
        pcl::copyPointCloud(*cloud_filtered, cluster_indices[i], *cluster);

        std::stringstream ss;
        ss << "cluster_" << i;

//        pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(cluster);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(cloud, rand()*255, rand()*255, rand()*255);
        viewer->addPointCloud<PointType> (cluster, single_color, ss.str().c_str());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str().c_str());
    }



    //    viewer->addPointCloud(cloud_filtered, "cloud");
    //    viewer->addPointCloudNormals<PointType, pcl::Normal>(cloud_filtered, cloud_normals, 100, 0.02, "normals");


    //    pcl::PointCloud<PointType>::Ptr cloud_vox(new pcl::PointCloud<PointType>());
    //    voxy->voxelToCloudZeroCrossing(cloud_vox);
    //
    //    // Create a KD-Tree
    //    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    //
    //    // Output has the PointNormal type in order to store the normals calculated by MLS
    //    pcl::PointCloud<XYZNormalType>::Ptr mls_points(new pcl::PointCloud<XYZNormalType>());
    //    // Init object (second point type is for the normals, even if unused)
    //    pcl::MovingLeastSquares<PointType, XYZNormalType> mls;
    //
    //    mls.setComputeNormals(true);
    //
    //    // Set parameters
    //    mls.setInputCloud(cloud_vox);
    //    mls.setPolynomialFit(true);
    //    mls.setSearchMethod(tree);
    //    mls.setSearchRadius(0.05);
    //    mls.setPolynomialOrder(2);
    //    //    mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointType,XYZNormalType>::SAMPLE_LOCAL_PLANE);
    //    //    mls.setUpsamplingRadius(0.005);
    //    //    mls.setUpsamplingStepSize(0.003);
    //    // Reconstruct
    //    mls.process(*mls_points);
    //
    //    pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
    //
    //    for (int i = 0; i < mls_points->points.size(); i++) {
    //        PointType p;
    //        pcl::PointNormal n = mls_points->points[i];
    //        p.x = n.x;
    //        p.y = n.y;
    //        p.z = n.z;
    //        p.r = 255;
    //        p.g = 255;
    //        p.b = 255;
    //
    //        scene->points.push_back(p);
    //    }
    //
    //
    //    viewer->removeAllPointClouds();
    //    viewer->addPointCloud(scene, "cloud_vox");

    //
    //        typedef pcl::PointNormal XYZNormalType;
    //    
    //        // Create a KD-Tree
    //        pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    //    
    //        // Output has the PointNormal type in order to store the normals calculated by MLS
    //        pcl::PointCloud<XYZNormalType>::Ptr mls_points(new pcl::PointCloud<XYZNormalType>());
    //        ;
    //    
    //        // Init object (second point type is for the normals, even if unused)
    //        pcl::MovingLeastSquares<PointType, XYZNormalType> mls;
    //    
    //        mls.setComputeNormals(true);
    //    
    //        // Set parameters
    //        mls.setInputCloud(cloud_vox);
    //        mls.setPolynomialFit(true);
    //        mls.setSearchMethod(tree);
    //        mls.setSearchRadius(0.05);
    //        mls.setPolynomialOrder(2);
    //        //    mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointType,XYZNormalType>::SAMPLE_LOCAL_PLANE);
    //        //    mls.setUpsamplingRadius(0.005);
    //        //    mls.setUpsamplingStepSize(0.003);
    //        // Reconstruct
    //        mls.process(*mls_points);
    //
    //    // Save output
    //    pcl::io::savePCDFile("bun0-mls.pcd", *mls_points);
    //
    //
    //    viewer->addPointCloud(cloud_vox, "cloud_vox");


    while (!viewer->wasStopped()) {
        viewer->spinOnce();
        cv::waitKey(10);
    }

    //    pcl::Poisson<XYZNormalType> poisson;
    //    poisson.setDepth (9);
    //    poisson.setInputCloud (mls_points);
    //    pcl::PolygonMesh mesh;
    //    poisson.reconstruct (mesh);
    //    pcl::io::saveVTKFile ("/home/daniele/Desktop/sreconstruc.vtk",mesh);

    //    pcl::io::savePLYFileASCII("/home/daniele/Desktop/mls.ply", *mls_points);
    //    pcl::io::savePLYFileASCII("/home/daniele/Desktop/output.ply", *cloud_vox);

    return (0);
}