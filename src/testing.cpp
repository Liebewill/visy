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
#include <pcl/surface/gp3.h>
#include <pcl/filters/fast_bilateral.h>

#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>

#include "Parameters.h"
#include "WillowDataset.h"
#include "Voxy.h"

typedef pcl::Normal PointNormalType;
typedef pcl::PointNormal PointWithNormalType;
pcl::PointCloud<PointType>::Ptr occupancy_cloud(new pcl::PointCloud<PointType>());

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
            }//            else if (this->pointValue(p.z - step / 2.0f) >= map_min_inliers) {
                //                planes_indices.push_back(i);
                //            } 
            else if (this->pointValue(p.z + step / 2.0f) >= map_min_inliers) {
                planes_indices.push_back(i);
            } else {
                filtered_indices.push_back(i);
            }

        }
    }
};

struct Palette {
    std::vector<Eigen::Vector3i> colors;
    int index = -1;

    Palette() {
        colors.push_back(Eigen::Vector3i(115, 255, 0));
        colors.push_back(Eigen::Vector3i(232, 159, 12));
        colors.push_back(Eigen::Vector3i(255, 0, 0));
        colors.push_back(Eigen::Vector3i(61, 12, 232));
        colors.push_back(Eigen::Vector3i(13, 255, 239));
    }

    Eigen::Vector3i getColor() {
        index++;
        index = index % colors.size();
        return colors[index];
    }
};

Palette palette;

visy::Parameters * parameters;
Eigen::Matrix4f adjust;
int current_index = 0;
visy::Voxy* voxy;
ElevatorMap* emap;
pcl::visualization::PCLVisualizer* viewer;
int viewport_1, viewport_2;
pcl::PointCloud<PointType>::Ptr full_cloud(new pcl::PointCloud<PointType>());
typedef pcl::PointNormal XYZNormalType;


std::string cloud_base_path = "/home/daniele/Desktop/TSDF_Dataset/Cups_Refined/";

void loadCloud(int index, pcl::PointCloud<PointType>::Ptr& cloud, Eigen::Matrix4f& t) {

    pcl::PointCloud<PointType>::Ptr cloud_raw(new pcl::PointCloud<PointType>());
    std::stringstream ss;
    ss << cloud_base_path;
    ss << index << ".pcd";
    if (pcl::io::loadPCDFile<PointType> (ss.str().c_str(), *cloud_raw) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");

    }

    float sd = parameters->getFloat("sd");
    float sr = parameters->getFloat("sr");

    if (sd < 0 || sr < 0) {
        cloud = cloud_raw;
    } else {
        pcl::FastBilateralFilter<PointType> bif;
        bif.setSigmaS(sd);
        bif.setSigmaR(sr);
        bif.setInputCloud(cloud_raw);
        bif.applyFilter(*cloud);
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

void slicerZ(pcl::PointCloud<PointType>::Ptr& cloud, std::vector<pcl::PointIndices>& slice_indices, float slice_size) {

    float min_z = 2.0f;
    float max_z = -2.0f;

    for (int i = 0; i < cloud->points.size(); i++) {
        PointType p = cloud->points[i];
        min_z = p.z < min_z ? p.z : min_z;
        max_z = p.z > max_z ? p.z : max_z;
    }

    slice_indices.clear();
    for (float z = min_z; z <= max_z; z += slice_size) {
        pcl::PointIndices indices;
        slice_indices.push_back(indices);
    }

    for (int i = 0; i < cloud->points.size(); i++) {
        PointType p = cloud->points[i];
        float index = floor((p.z - min_z) / slice_size);
        slice_indices[(int) index].indices.push_back(i);
    }

}

float coarseArea(pcl::PointCloud<PointType>::Ptr& cloud) {
    Eigen::Vector4f centroid4;
    Eigen::Vector3f centroid;
    pcl::compute3DCentroid(*cloud, centroid4);
    centroid <<
            centroid4(0),
            centroid4(1),
            centroid4(2);
    float area = 0.0f;
    float max_x = -200.0f;
    float min_x = 200.0f;
    float max_y = -200.0f;
    float min_y = 200.0f;
    for (int i = 0; i < cloud->points.size(); i++) {
        PointType p = cloud->points[i];
        max_x = p.x > max_x ? p.x : max_x;
        max_y = p.y > max_y ? p.y : max_y;
        min_y = p.y < min_y ? p.y : min_y;
        min_x = p.x < min_x ? p.x : min_x;

    }
    return sqrt((max_x - min_x)*(max_x - min_x)) * sqrt((max_y - min_y)*(max_y - min_y));
}

struct Grasp {
    Eigen::Vector3f p1;
    Eigen::Vector3f p2;
    Eigen::Vector3f pw;
    Eigen::Vector3f center;
    float mag;
};

void sliceGraspPoints(pcl::PointCloud<PointType>::Ptr& slice, std::vector<Grasp>& grasps, float wrist_distance = 0.15f, float gripper_size = 0.05f) {
    Eigen::Vector4f centroid4;
    Eigen::Vector3f centroid;
    pcl::compute3DCentroid(*slice, centroid4);
    centroid <<
            centroid4(0),
            centroid4(1),
            centroid4(2);

    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(slice);

    pcl::KdTreeFLANN<PointType> tree_occupancy;
    tree_occupancy.setInputCloud(occupancy_cloud);
    std::vector<int> occ_found_indices;
    std::vector<float> occ_found_distances;


    Eigen::Vector3f dir;
    Eigen::Vector3f search_p;
    std::vector<int> found_indices;
    std::vector<float> found_distances;

    for (int i = 0; i < slice->points.size(); i++) {
        PointType pp = slice->points[i];
        Eigen::Vector3f p;
        p << pp.x, pp.y, pp.z;
        dir = p - centroid;
        dir = -dir;

        for (float perc = 0.0f; perc <= 2.0f; perc += 0.2f) {
            search_p = centroid + dir*perc;
            found_indices.clear();
            found_distances.clear();
            PointType sp;
            visy::tools::convertPoint3D(sp, search_p, true);
            search_p(2) = p(2);
            int n = tree.radiusSearch(sp, 0.005f, found_indices, found_distances);



            if (n > 0) {

                Eigen::Vector3f center = 0.5f * (p + search_p);
                Eigen::Vector3f side = p - center;
                Eigen::Vector3f wrist = p - center;
                Eigen::Vector3f axis = p - search_p;
                PointType sp;
                axis = axis.normalized();


                Eigen::AngleAxis<float> aa(M_PI / 2.0f, Eigen::Vector3f(0, 0, 1.0f));


                side = aa*side;
                side = side.normalized();
                wrist = center - side * wrist_distance;


                visy::tools::convertPoint3D(sp, wrist, true);
                int on = tree_occupancy.radiusSearch(sp, gripper_size, occ_found_indices, occ_found_distances);
                if (on > 0) {
                    wrist = center + side * wrist_distance;
                    visy::tools::convertPoint3D(sp, wrist, true);
                    int on = tree_occupancy.radiusSearch(sp, gripper_size, occ_found_indices, occ_found_distances);
                    if (on > 0) {
                        wrist = center + Eigen::Vector3f(0, 0, 1.0f) * wrist_distance;
                        visy::tools::convertPoint3D(sp, wrist, true);
                        int on = tree_occupancy.radiusSearch(sp, gripper_size, occ_found_indices, occ_found_distances);
                        if (on > 0) {
                            //                            continue;
                        }
                    }
                }

                Grasp g;
                g.p1 = p;
                g.p2 = search_p;
                g.pw = wrist;
                g.center = center;
                g.mag = (g.p1 - g.p2).norm();
                grasps.push_back(g);
                //                break;
            }
        }
    }

}

//void elevatorPlanesCheck(
//        pcl::PointCloud<PointType>::Ptr& cloud,
//        pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
//        std::vector<int>& filtered_indices,
//        std::vector<int>& planes_indices,
//        float max_angle,
//        ElevatorMap& emap,
//        int map_min_inliers
//        ) {
//
//    Eigen::Vector3f normal;
//    Eigen::Vector3f gravity_neg(0, 0, 1);
//
//
//    for (int i = 0; i < cloud_normals->points.size(); i++) {
//        pcl::Normal n = cloud_normals->points[i];
//        PointType p = cloud->points[i];
//        normal(0) = n.normal_x;
//        normal(1) = n.normal_y;
//        normal(2) = n.normal_z;
//
//        float angle = acos(normal.dot(gravity_neg));
//        if (angle <= max_angle * M_PI / 180.0f) {
//            emap.pinPoint(p.z);
//        }
//    }
//
//    for (int i = 0; i < cloud->points.size(); i++) {
//        PointType p = cloud->points[i];
//
//        if (emap.pointValue(p.z) >= map_min_inliers) {
//            planes_indices.push_back(i);
//        } else {
//            filtered_indices.push_back(i);
//        }
//
//    }
//}

void showCloud(pcl::PointCloud<PointType>::Ptr cloud, double r, double g, double b, float size, std::string name) {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(cloud, r, g, b);
    viewer->addPointCloud<PointType> (cloud, single_color, name.c_str());
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name.c_str());
}

void computeNormals(pcl::PointCloud<PointType>::Ptr& cloud, pcl::PointCloud<PointNormalType>::Ptr& cloud_normals) {
    pcl::NormalEstimationOMP<PointType, PointNormalType> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType> ());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*cloud_normals);
}

void filterCloudMLS(pcl::PointCloud<PointType>::Ptr& cloud_in, pcl::PointCloud<PointType>::Ptr& cloud_out, float radius = 0.05) {
    // Create a KD-Tree
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<XYZNormalType>::Ptr mls_points(new pcl::PointCloud<XYZNormalType>());
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointType, XYZNormalType> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud_in);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.setPolynomialOrder(2);
    //    mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointType,XYZNormalType>::SAMPLE_LOCAL_PLANE);
    //    mls.setUpsamplingRadius(0.005);
    //    mls.setUpsamplingStepSize(0.003);
    // Reconstruct
    mls.process(*mls_points);

    cloud_out->points.clear();
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

void processCloud(visy::Voxy* voxy, int index, pcl::PointCloud<PointType>::Ptr& cloud_out, pcl::PointCloud<PointNormalType>::Ptr& cloud_normals) {

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>());
    Eigen::Matrix4f t;
    loadCloud(index, cloud, t);

    t = t*adjust;

    pcl::transformPointCloud(*cloud, *cloud_trans, t);
    Eigen::Vector3f pov(t(0, 3), t(1, 3), t(2, 3));

    boost::posix_time::ptime time_start(boost::posix_time::microsec_clock::local_time());

    std::cout << "Cloud trans: " << cloud_trans->points.size() << std::endl;
    voxy->addPointCloud(cloud_trans, pov);

    boost::posix_time::ptime time_end(boost::posix_time::microsec_clock::local_time());
    boost::posix_time::time_duration duration(time_end - time_start);

    pcl::PointCloud<PointType>::Ptr cloud_vox(new pcl::PointCloud<PointType>());
    voxy->voxelToCloudZeroCrossing(cloud_vox);
    std::cout << "Voxy update time: " << duration << std::endl;

    /**FILTER */
    //filterCloudMLS(cloud_vox, cloud_out);
    cloud_out = cloud_vox;

    std::cout << "Vox size: " << cloud_vox->points.size() << std::endl;
    /** NORMALS */
    computeNormals(cloud_out, cloud_normals);
}

std::string buildNameWithNumber(std::string name, int index) {
    std::stringstream ss;
    ss << name << "_" << index;
    return ss.str();
}

void buildClusters(
        pcl::PointCloud<PointType>::Ptr& cloud,
        std::vector<pcl::PointIndices>& cluster_indices) {
    pcl::search::KdTree<PointType>::Ptr tree2(new pcl::search::KdTree<PointType>);
    tree2->setInputCloud(cloud);


    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(0.04); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(250000);
    ec.setSearchMethod(tree2);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

void buildMesh(pcl::PointCloud<PointType>::Ptr& cloud, pcl::PolygonMesh& triangles) {


    pcl::PointCloud<NormalType>::Ptr cloud_normals(new pcl::PointCloud<NormalType>());
    computeNormals(cloud, cloud_normals);

    pcl::PointCloud<PointWithNormalType>::Ptr cloud_with_normals(new pcl::PointCloud<PointWithNormalType>);
    for (int i = 0; i < cloud->points.size(); i++) {
        PointWithNormalType pn;
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

    pcl::search::KdTree<PointWithNormalType>::Ptr tree2(new pcl::search::KdTree<PointWithNormalType>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<PointWithNormalType> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(parameters->getFloat("mesh_distance"));

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    if (parameters->getFloat("mesh_normals") > 0.0f) {
        gp3.setNormalConsistency(true);
    } else {
        gp3.setNormalConsistency(false);
    }

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    // Additional vertex information
    //    std::vector<int> parts = gp3.getPartIDs();
    //    std::vector<int> states = gp3.getPointStates();

}

void reduceCloud(pcl::PointCloud<PointType>::Ptr& cloud, pcl::PointCloud<PointType>::Ptr& cloud_filtered, float radius = 0.05) {
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud);
    float leaf_size = radius;
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_filtered);
}

void buildPlane(pcl::PointCloud<PointType>::Ptr& cloud_planes, pcl::PointCloud<PointType>::Ptr& cloud_flat) {

    float max_x = -2.0f;
    float min_x = 2.0f;
    float max_y = -2.0f;
    float min_y = 2.0f;
    float max_z = -2.0f;
    float min_z = 2.0f;
    for (int i = 0; i < cloud_planes->points.size(); i++) {
        PointType p = cloud_planes->points[i];

        max_x = p.x > max_x ? p.x : max_x;
        max_y = p.y > max_y ? p.y : max_y;
        max_z = p.z > max_z ? p.z : max_z;
        min_x = p.x < min_x ? p.x : min_x;
        min_y = p.y < min_y ? p.y : min_y;
        min_z = p.z < min_z ? p.z : min_z;

    }

    float z = (max_z + min_z) / 2.0f;
    float step = 0.01f;
    for (float x = min_x; x <= max_x; x += step) {
        for (float y = min_y; y <= max_y; y += step) {
            PointType p;
            p.x = x;
            p.y = y;
            p.z = z;
            cloud_flat->points.push_back(p);
        }
    }
}

std::vector<pcl::PointCloud<PointType>::Ptr> current_clusters;

void nextRound() {
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_planes(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_without_planes(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointNormalType>::Ptr cloud_normals(new pcl::PointCloud<PointNormalType>);
    std::vector<int> withoutplanes_indices;
    std::vector<int> planes_indices;

    processCloud(
            voxy,
            current_index,
            cloud,
            cloud_normals
            );
    current_index += parameters->getFloat("step");

    //FILTERING
    pcl::PointCloud<PointType>::Ptr cloud_reduced(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());
    reduceCloud(cloud, cloud_reduced, 0.005);
    filterCloudMLS(cloud_reduced, cloud_filtered, 0.02);
    computeNormals(cloud_filtered, cloud_normals);

    occupancy_cloud = cloud_filtered;

    //CLUSTERING
    emap->planesCheck(
            cloud_filtered,
            cloud_normals,
            withoutplanes_indices,
            planes_indices,
            8.0f,
            6000);
    pcl::copyPointCloud(*cloud_filtered, planes_indices, *cloud_planes);
    pcl::copyPointCloud(*cloud_filtered, withoutplanes_indices, *cloud_without_planes);

    reduceCloud(cloud_planes, cloud_planes, 0.005f);
    filterCloudMLS(cloud_planes, cloud_planes, 0.02f);

    std::vector<pcl::PointIndices> cluster_indices;
    buildClusters(cloud_without_planes, cluster_indices);


    //VIEW

    viewer->removeAllPointClouds();

    //    showCloud(cloud_filtered, 255, 255, 255, 1, "cloud");
    if (cloud_planes->points.size() > 0) {
        pcl::PointCloud<PointType>::Ptr flat_planes(new pcl::PointCloud<PointType>());
        //        buildPlane(cloud_planes, flat_planes);
        pcl::PolygonMesh plane_triangles;
        buildMesh(cloud_planes, plane_triangles);
        viewer->addPolygonMesh(plane_triangles, "planes");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5f, 0.5f, 0.5f, "planes");
    }
    //    showCloud(flat_planes, 125, 125, 125, 1, "planes");

    Palette palette;
    current_clusters.clear();
    for (int i = 0; i < cluster_indices.size(); i++) {
        pcl::PointIndices points = cluster_indices[i];
        pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*cloud_without_planes, points.indices, *cluster);
        current_clusters.push_back(cluster);
        Eigen::Vector3i color;

        if (cloud_planes->points.size() > 0) {
            color = palette.getColor();
        } else {
            color << 125, 125, 125;
        }

        std::stringstream ss;
        ss << "cluister_" << i;
        //        showCloud(cluster, color(0), color(1), color(2), 1, ss.str().c_str());
        pcl::PolygonMesh triangles;

        buildMesh(cluster, triangles);
        viewer->addPolygonMesh(triangles, ss.str().c_str());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color(0) / 255.0, color(1) / 255.0, color(2) / 255.0, ss.str().c_str());

    }

    //    pcl::PolygonMesh triangles;
    //    
    //    buildMesh(cloud_without_planes, triangles);
    //    viewer->addPolygonMesh(triangles, "mesh");
    //    viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0, "mesh" );
    //    
}

bool show_gripper = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
        void* viewer_void) {
    if (event.getKeySym() == "v" && event.keyDown()) {
        std::cout << "OK!" << std::endl;
        //addPointCloudToVox(voxy,current_index);
        //        if (current_index < 60) {
        //            addPointCloudToViewer(current_index);
        nextRound();
        //        }
    }

    if (event.getKeySym() == "l" && event.keyDown()) {
        show_gripper = true;
    }
    if (event.getKeySym() == "k" && event.keyDown()) {
        for (int i = 0; i < current_clusters.size(); i++) {
            std::vector<pcl::PointIndices> slice_indices;
            slicerZ(current_clusters[i], slice_indices, parameters->getFloat("slice"));

            std::vector<Grasp> grasps;
            pcl::PointCloud<PointType>::Ptr center_slice(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*current_clusters[i], slice_indices[slice_indices.size() / 2].indices, *center_slice);
            sliceGraspPoints(center_slice, grasps);

            int index = -1;
            float max_mag = 2222220.0f;
            for (int i = 0; i < grasps.size(); i++) {
                if (grasps[i].mag < max_mag) {
                    max_mag = grasps[i].mag;
                    index = i;
                }

            }
            if (index == -1)continue;
            Palette pal;
            PointType p1, p2, pw, pc;
            visy::tools::convertPoint3D(p1, grasps[index].p1, true);
            visy::tools::convertPoint3D(p2, grasps[index].p2, true);
            visy::tools::convertPoint3D(pw, grasps[index].pw, true);
            visy::tools::convertPoint3D(pc, grasps[index].center, true);
            Eigen::Vector3i color = pal.getColor();
            std::stringstream ss;
            ss.str("");
            ss << "grasp_" << index << "_p1";

            viewer->addSphere(p1, 0.01f, 0, 0.8f, 0.8f, ss.str().c_str());

            ss.str("");
            ss << "grasp_" << index << "_p2";
            viewer->addSphere(p2, 0.01f, 0, 0.8f, 0.8f, ss.str().c_str());

            ss.str("");
            ss << "grasp_" << index << "_l1";
            //            viewer->addArrow(pw, pc, 0.8f, 0.8f, 0, false, ss.str().c_str());

            if (show_gripper) {
                ss.str("");
                ss << "grasp_" << index << "_l1";
                viewer->addLine(pw, p1, 0.8f, 0.8f, 0.2f, ss.str().c_str());
                ss.str("");
                ss << "grasp_" << index << "_l2";
                viewer->addLine(pw, p2, 0.8f, 0.8f, 0.2f, ss.str().c_str());

                ss.str("");
                ss << "grasp_" << index << "_pw";
                viewer->addSphere(pw, 0.03f, 0.8f, 0.8f, 0.2f, ss.str().c_str());
            }
            //            pcl::PointCloud<PointType>::Ptr grasp_cloud(new pcl::PointCloud<PointType>());
            //            grasp_cloud->points.push_back(p1);
            //            grasp_cloud->points.push_back(p2);
            //            showCloud(grasp_cloud, color(0), color(1), color(2), 25, ss.str().c_str());
        }
    }
}

int
main(int argc, char** argv) {
    /** PARAMETERS */
    parameters = new visy::Parameters(argc, argv);
    parameters->putFloat("sigma");
    parameters->putFloat("step");
    parameters->putFloat("offx", -0.3f);
    parameters->putFloat("offy", 0.5f);
    parameters->putFloat("offz", 1.0f);
    parameters->putFloat("slice", 0.01f);
    parameters->putFloat("sd", 3.0f);
    parameters->putFloat("sr", 0.1f);
    parameters->putFloat("voxy_size", 256.0f);
    parameters->putFloat("voxy_edge", 1.0f);
    parameters->putFloat("mesh_distance", 0.025f);
    parameters->putFloat("mesh_normals", 1.0f);
    parameters->putFloat("lvl_max", 2.0f);
    parameters->putFloat("lvl_step", 0.005f);
    parameters->putFloat("lvl_offset", 1.0f);

    /* VIEWER */
    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);


    //system("espeak -v it -s 80 \"ho iniziato\"");
    /* VOXY */
    int size = parameters->getFloat("voxy_size");
    int full_size = size * size*size;
    double step = 0.5f / (double) size;
    double sigma = parameters->getFloat("sigma");
    std::cout << "SIGMA: " << sigma << std::endl;
    Eigen::Vector3f offset(
            parameters->getFloat("offx"),
            parameters->getFloat("offy"),
            parameters->getFloat("offz")
            );

    voxy = new visy::Voxy(size, parameters->getFloat("voxy_edge"), sigma, offset);

    adjust <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    /** ELEVATOR MAP */
    emap = new ElevatorMap(
            parameters->getFloat("lvl_max"),
            parameters->getFloat("lvl_step"),
            parameters->getFloat("lvl_offset")
            );

    boost::posix_time::ptime time_start, time_end;
    boost::posix_time::time_duration duration;

    //    std::vector<int> ics;
    //    ics.push_back(50);
    //    ics.push_back(160);
    //    for(int i = 0; i < ics.size(); i++){
    //    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    //        pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>());
    //        Eigen::Matrix4f t;
    //        
    //        loadCloud(ics[i],cloud,t);
    //        t = t*adjust;
    //        pcl::transformPointCloud(*cloud, *cloud_trans, t);
    //        std::stringstream ss;
    //        ss<<"c_"<<i;
    //        viewer->addPointCloud(cloud_trans,ss.str());
    //    }


    //    for (int index = 0; index <= 166; index += 10 ) {
    //        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    //        pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>());
    //        Eigen::Matrix4f t;
    //        loadCloud(index, cloud, t);
    //
    //        pcl::transformPointCloud(*cloud, *cloud_trans, t);
    //        Eigen::Vector3f pov(t(0, 3), t(1, 3), t(2, 3));
    //
    //        time_start = boost::posix_time::microsec_clock::local_time();
    //
    //        std::cout << "Cloud trans: " << cloud_trans->points.size() << std::endl;
    //        voxy->addPointCloud(cloud_trans, pov);
    //
    //        time_end = boost::posix_time::microsec_clock::local_time();
    //        duration = time_end - time_start;
    //        std::cout << "Voxy update time: " << duration << std::endl;
    //
    //    }
    //    pcl::PointCloud<PointType>::Ptr cloud_isosurface(new pcl::PointCloud<PointType>());
    //
    //    voxy->voxelToCloudZeroCrossing(cloud_isosurface);
    //
    //    showCloud(cloud_isosurface, 255, 255, 255, 1, "isosurface");


    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }


    pcl::PointCloud<PointType>::Ptr cloud_vox(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_down(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>());
    voxy->voxelToCloudZeroCrossing(cloud_vox);

    std::cout << "Filtering cloud" << std::endl;
    //  filterCloudMLS(cloud_vox, cloud_out);
    pcl::PointCloud<int> sampled_indices;

    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud_vox);
    float leaf_size = 0.005f;
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_down);
    //  filterCloudMLS(cloud_down, cloud_out);
    cloud_out = cloud_down;
    cloud_vox->width = 1;
    cloud_vox->height = cloud_vox->points.size();

    cloud_out->width = 1;
    cloud_out->height = cloud_out->points.size();


    std::stringstream ss;
    ss << "/home/daniele/Desktop/raw_" << sigma << ".pcd";
    pcl::io::savePCDFile(ss.str(), *cloud_vox);
    pcl::io::savePCDFile("/home/daniele/Desktop/filtered.pcd", *cloud_out);

    for (unsigned int i = 0; i < current_clusters.size(); i++) {
        ss.str("");
        ss << "/home/daniele/Desktop/raw_cluster_" << i << ".pcd";
        pcl::io::savePCDFile(ss.str(), *(current_clusters[i]));
    }

    return (0);
}