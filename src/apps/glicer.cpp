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
#include "tools.h"

typedef pcl::PointNormal PointNormalType;
typedef pcl::PointNormal XYZNormalType;

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
    float mag;
};

void sliceGraspPoints(pcl::PointCloud<PointType>::Ptr& slice, std::vector<Grasp>& grasps) {
    Eigen::Vector4f centroid4;
    Eigen::Vector3f centroid;
    pcl::compute3DCentroid(*slice, centroid4);
    centroid <<
            centroid4(0),
            centroid4(1),
            centroid4(2);

    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(slice);

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

        for (float perc = 2.0f; perc > 0.0f; perc -= 0.2f) {
            search_p = centroid + dir*perc;
            found_indices.clear();
            found_distances.clear();
            PointType sp;
            visy::tools::convertPoint3D(sp, search_p, true);
            int n = tree.radiusSearch(sp, 0.005f, found_indices, found_distances);
            if (n > 0) {
                Grasp g;
                g.p1 = p;
                g.p2 = search_p;
                g.mag = (g.p1 - g.p2).norm();
                grasps.push_back(g);
                break;
            }
        }
    }

}

int
main(int argc, char** argv) {

    /** PARAMETERS */
    parameters = new visy::Parameters(argc, argv);
    parameters->putString("source");
    parameters->putFloat("leaf");
    parameters->putFloat("slice", 0.01f);
    parameters->putFloat("polinomial");

    /* VIEWER */
    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");
    //    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);


    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr centroids(new pcl::PointCloud<PointType>());


    std::string file = parameters->getString("source");
    if (pcl::io::loadPCDFile<PointType> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    }



    std::vector<pcl::PointIndices> slice_indices;
    slicerZ(cloud, slice_indices, parameters->getFloat("slice"));

    Palette palette;
    std::stringstream ss;
    for (int i = 0; i < slice_indices.size(); i++) {
        pcl::PointCloud<PointType>::Ptr slice(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*cloud, slice_indices[i].indices, *slice);
        Eigen::Vector3i color = palette.getColor();
        ss.str("");
        ss << "slice_" << i;
        showCloud(slice, color(0), color(1), color(2), 1, ss.str().c_str());

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*slice, centroid);
        PointType p;
        p.x = centroid(0);
        p.y = centroid(1);
        p.z = centroid(2);
        centroids->points.push_back(p);

        std::cout << "Area slice " << i << " :" << coarseArea(slice) << std::endl;
    }

    std::vector<Grasp> grasps;
    pcl::PointCloud<PointType>::Ptr center_slice(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*cloud, slice_indices[slice_indices.size() / 2].indices, *center_slice);
    sliceGraspPoints(center_slice, grasps);

    std::cout << "Grasp couples: " << grasps.size() << std::endl;

    Palette pal;
    int index = -1;
    float max_mag = 2222220.0f;
    for (int i = 0; i < grasps.size(); i++) {
        if(grasps[i].mag < max_mag){
            max_mag = grasps[i].mag;
            index = i;
        }
        
    }
//    for (int i = 0; i < grasps.size(); i++) {
        PointType p1, p2;
        visy::tools::convertPoint3D(p1, grasps[index].p1, true);
        visy::tools::convertPoint3D(p2, grasps[index].p2, true);
        Eigen::Vector3i color = pal.getColor();
        ss.str("");
        ss << "grasp_" << index << "_p1";
        pcl::PointCloud<PointType>::Ptr grasp_cloud(new pcl::PointCloud<PointType>());
        grasp_cloud->points.push_back(p1);
        grasp_cloud->points.push_back(p2);
        showCloud(grasp_cloud, color(0), color(1), color(2), 25, ss.str().c_str());

//    }

    showCloud(centroids, 255, 0, 0, 8, "centroids");

    //    viewer->addPointCloud(cloud, "cloud", v0);


    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }


    return (0);
}