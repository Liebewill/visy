#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/bilateral.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>

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

void computeNormals(pcl::PointCloud<PointType>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals) {
    pcl::NormalEstimationOMP<PointType, pcl::Normal> ne;
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
        if (iz<this->size) {
            this->map[iz]++;
        }
    }

    double pointValue(double z) {
        int iz = floor(z / step);
        iz += floor(this->offset / step);
        if (iz<this->size) {
            return this->map[iz];
        }
        return -1;
    }
};

void elevatorPlanesCheck(
        pcl::PointCloud<PointType>::Ptr& cloud,
        pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
        std::vector<int>& filtered_indices,
        std::vector<int>& planes_indices,
        float max_angle,
        ElevatorMap& emap,
        int map_min_inliers
        ) {

    Eigen::Vector3f normal;
    Eigen::Vector3f gravity_neg(0, 0, 1);

    std::vector<int> full_indices;

    for (int i = 0; i < cloud_normals->points.size(); i++) {
        pcl::Normal n = cloud_normals->points[i];
        PointType p = cloud->points[i];
        normal(0) = n.normal_x;
        normal(1) = n.normal_y;
        normal(2) = n.normal_z;

        float angle = acos(normal.dot(gravity_neg));
        if (angle > max_angle * M_PI / 180.0f) {

        } else {
            full_indices.push_back(i);
            emap.pinPoint(p.z);
        }
    }

    for (int i = 0; i < cloud->points.size(); i++) {
        PointType p = cloud->points[i];

        if (emap.pointValue(p.z) >= map_min_inliers) {
            planes_indices.push_back(i);
        } else {
            filtered_indices.push_back(i);
        }

    }

}

void buildClusters(
        pcl::PointCloud<PointType>::Ptr& cloud,
        std::vector<pcl::PointIndices>& cluster_indices) {
    pcl::search::KdTree<PointType>::Ptr tree2(new pcl::search::KdTree<PointType>);
    tree2->setInputCloud(cloud);


    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(500);
    ec.setMaxClusterSize(250000);
    ec.setSearchMethod(tree2);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

void showCloud(pcl::PointCloud<PointType>::Ptr cloud, double r, double g, double b, float size, std::string name) {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(cloud, r, g, b);
    viewer->addPointCloud<PointType> (cloud, single_color, name.c_str());
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name.c_str());
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

    /* VIEWER */
    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

    /* VOXY */
    int size = 256;
    int full_size = size * size*size;
    double step = 2.0f / (double) size;
    Eigen::Vector3f offset(0.0, 1.0, 1.0);
    voxy = new visy::Voxy(size, 2.0, 0.01f, offset);

    /** ELEVATOR MAP */
    ElevatorMap emap(2.0, 0.1, 1.0);


    /** LOAD CLOUD */
    for (int i = 0; i <= 50; i += 5) {
        int index = 20;
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>());
        Eigen::Matrix4f t;
        loadCloud(i, cloud, t);
        t = t*adjust;
        pcl::transformPointCloud(*cloud, *cloud_trans, t);
        Eigen::Vector3f pov(t(0, 3), t(1, 3), t(2, 3));

        voxy->addPointCloud(cloud_trans, pov);


        /** NORMALS */
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
        std::vector<int> filtered_indices;
        std::vector<int> planes_indices;

        computeNormals(cloud_trans, cloud_normals);

        elevatorPlanesCheck(cloud_trans, cloud_normals, filtered_indices, planes_indices, 30.0, emap, 15000);

        pcl::copyPointCloud(*cloud_trans, filtered_indices, *cloud_filtered);


        //    viewer->addPointCloud(cloud_filtered, "cloud_vox");


        std::vector<pcl::PointIndices> cluster_indices;
        buildClusters(cloud_filtered, cluster_indices);



        for (int i = 0; i < cluster_indices.size(); i++) {
            pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>);
            pcl::copyPointCloud(*cloud_filtered, cluster_indices[i], *cluster);
            voxy->addCluster(cluster);
        }

    }

    pcl::PointCloud<PointType>::Ptr cloud_vox(new pcl::PointCloud<PointType>());
    std::vector<int> labels;
    voxy->voxelToCloud(cloud_vox, labels);

    for (int j = -1; j < voxy->labelsCount(); j++) {
        pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>);
        std::vector<int> indices;
        for (int i = 0; i < labels.size(); i++) {
            if (labels[i] == j) {
                indices.push_back(i);
            }
        }
        pcl::copyPointCloud(*cloud_vox, indices, *cluster);
        std::stringstream ss;
        ss << "cluster_" << j;
        showCloud(cluster, rand()*255, rand()*255, rand()*255, 3, ss.str());
    }

    //showCloud(cloud_vox,255,255,255,2,"VOX");

    //
    //
    //    //
    //    for (int i = 0; i < cluster_indices.size(); i++) {
    //        pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>);
    //        pcl::copyPointCloud(*cloud_filtered, cluster_indices[i], *cluster);
    //        std::stringstream ss;
    //        ss << "cluster_" << i;
    //        showCloud(cluster, rand()*255, rand()*255, rand()*255, 2, ss.str());
    //    }
    //
    //    pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>);
    //    pcl::copyPointCloud(*cloud_trans, planes_indices, *cluster);
    //    showCloud(cluster, rand()*255, rand()*255, rand()*255, 2, "planes");



    //    viewer->addPointCloud(cloud_filtered, "cloud");
    //    viewer->addPointCloudNormals<PointType, pcl::Normal>(cloud_filtered, cloud_normals, 100, 0.02, "normals");


    //        pcl::PointCloud<PointType>::Ptr cloud_vox(new pcl::PointCloud<PointType>());
    //        voxy->voxelToCloudZeroCrossing(cloud_vox);
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