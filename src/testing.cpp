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


#include "Parameters.h"
#include "WillowDataset.h"

/** SCENE */
std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
std::vector<visy::extractors::KeyPoint3D> scene_keypoints_seletected;
std::vector<visy::extractors::KeyPoint3D> scene_keypoints_parallels_all;
std::vector<visy::extractors::KeyPoint3D> scene_keypoints_parallels;
int scene_keypoints_parallels_best = -1;
std::vector<cv::Point2f> scene_keypoints_seletected_parallels;
int scene_keypoint_selected_index = -1;
cv::Mat scene_descriptor;
cv::Mat scene_rgb, scene_rgb_full;
pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr scene_cloud_filtered(new pcl::PointCloud<PointType>());

visy::Parameters * parameters;

void gravityVector(pcl::PointCloud<PointType>::Ptr cloud, cv::Point3f& gravity_vector) {
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointType> vg;
    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_filtered_orig(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_f(new pcl::PointCloud<PointType>);


    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*
    //    cloud_filtered = cloud;
    pcl::copyPointCloud(*cloud_filtered, *cloud_filtered_orig);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointType> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<PointType>::Ptr cloud_plane(new pcl::PointCloud<PointType> ());
    std::vector<pcl::PointCloud<PointType>::Ptr> cloud_planes;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    std::vector<std::vector<int> > planes_inliers;
    std::vector<std::vector<float> > planes_coeff;
    std::vector<cv::Point3f > planes_normals;

    int i = 0, nr_points = (int) cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.3 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        pcl::PointCloud<PointType>::Ptr cloud_plane_iter(new pcl::PointCloud<PointType> ());
        pcl::copyPointCloud(*cloud_filtered, inliers->indices, *cloud_plane_iter);
        cloud_planes.push_back(cloud_plane_iter);


        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);


        planes_coeff.push_back(coefficients->values);
        planes_inliers.push_back(inliers->indices);

        cv::Point3f n(
                coefficients->values[0],
                coefficients->values[1],
                coefficients->values[2]
                );
        planes_normals.push_back(n);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }


    for (int i = 0; i < planes_coeff.size(); i++) {
        pcl::PointCloud<PointType>::Ptr cloud_plane(new pcl::PointCloud<PointType> ());
        pcl::copyPointCloud(*cloud_filtered, planes_inliers[i], *cloud_plane);
        std::stringstream ss;
    }

    float max_d = 10.0f * (M_PI / 180.0f);

    std::vector<int> similar_planes(planes_coeff.size());
    std::fill(similar_planes.begin(), similar_planes.end(), -1);
    for (int i = 0; i < planes_coeff.size(); i++) {
        cv::Point3f p_source = planes_normals[i];
        p_source = p_source * (1.0f / cv::norm(p_source));
        for (int j = 0; j < planes_coeff.size(); j++) {
            if (i != j) {
                cv::Point3f p_target = planes_normals[j];
                p_target = p_target * (1.0f / cv::norm(p_target));
                float pp = acos(p_source.dot(p_target));

                if (pp < max_d) {
                    similar_planes[i] = j;
                }

            }
        }
    }

    std::vector<int> planes_full_count;
    std::vector<cv::Point3f> planes_full_normals;
    std::vector<bool> graph_visit(similar_planes.size());
    std::fill(graph_visit.begin(), graph_visit.end(), false);
    bool full = false;
    while (!full) {
        int start_index = -1;
        int ref_index = -1;
        for (int i = 0; i < similar_planes.size(); i++) {
            full = true;
            if (!graph_visit[i]) {
                start_index = i;
                ref_index = i;
                graph_visit[i] = true;
                full = false;
                break;
            }
        }
        if (start_index == -1)continue;
        int counter = planes_inliers[start_index].size();
        bool visited = false;
        while (!visited) {
            start_index = similar_planes[start_index];
            if (start_index != -1 && !graph_visit[start_index]) {
                graph_visit[start_index] = true;
                counter += planes_inliers[start_index].size();
            } else {
                visited = true;
            }
        }
        planes_full_count.push_back(counter);
        planes_full_normals.push_back(cv::Point3f(
                planes_coeff[ref_index][0],
                planes_coeff[ref_index][1],
                planes_coeff[ref_index][2]
                ));

    }

    int max_plane = -1;
    int max_plane_count = 0;
    for (int i = 0; i < planes_full_count.size(); i++) {
        if (planes_full_count[i] > max_plane_count) {
            max_plane_count = planes_full_count[i];
            max_plane = i;
        }
    }

    gravity_vector.x = planes_coeff[max_plane][0];
    gravity_vector.y = planes_coeff[max_plane][1];
    gravity_vector.z = planes_coeff[max_plane][2];
}

int
main(int argc, char** argv) {
    /** PARAMETERS */
    parameters = new visy::Parameters(argc, argv);
    parameters->putFloat("gc_th");
    parameters->putFloat("gc_size");
    parameters->putFloat("down");
    parameters->putString("dataset");
    parameters->putString("detector");
    parameters->putString("model");
    parameters->putString("sizes");
    parameters->putInt("set");
    parameters->putInt("scene");
    parameters->putInt("nbin");
    parameters->putInt("occlusion");
    parameters->putBool("x");
    parameters->putBool("f");
    parameters->putFloat("f_th");
    parameters->putFloat("quantum");
    parameters->putFloat("edge_th");

    /** DATASET */
    std::cout << "Building Dataset: " << parameters->getString("dataset") << std::endl;
    visy::dataset::WillowDataset dataset(parameters->getString("dataset"));


    pcl::visualization::PCLVisualizer* viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");



    int set_number = parameters->getInt("set");
    int scene_number = parameters->getInt("scene");
    std::vector<visy::dataset::Annotation> annotations;
    dataset.loadScene(set_number, scene_number, cloud, scene_rgb);




    pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>);
    pcl::PointCloud<NormalType>::Ptr gravity_normals_cloud(new pcl::PointCloud<NormalType>);


    pcl::IntegralImageNormalEstimation<PointType, NormalType> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*scene_normals);


    cv::Point3f gravity;
    gravityVector(cloud, gravity);



    for (int i = 0; i < cloud->points.size(); i++) {
        NormalType n;
        n.normal_x = gravity.x;
        n.normal_y = gravity.y;
        n.normal_z = gravity.z;
        gravity_normals_cloud->points.push_back(n);
    }

    Eigen::Vector3f zero;
    zero << 0, 0, 0;

    Eigen::Vector3f z;
    z << 0, 0, 1;
    Eigen::Vector3f x;
    x << 1, 0, 0;
    Eigen::Vector3f y;
    y << 0, 1, 0;


    Eigen::Vector3f gz;
    gz << gravity.x, gravity.y, gravity.z;

    Eigen::Vector3f gx;
    gx << 1, 0, 0;

    Eigen::Vector3f gy = gz.cross(gx);


    visy::tools::draw3DVector(*viewer, zero, x, 1, 0, 0, "x");
    visy::tools::draw3DVector(*viewer, zero, y, 0, 1, 0, "y");
    visy::tools::draw3DVector(*viewer, zero, z, 0, 0, 1, "z");
    visy::tools::draw3DVector(*viewer, zero, gx, 1, 0, 0, "gx");
    visy::tools::draw3DVector(*viewer, zero, gy, 0, 1, 0, "gy");
    visy::tools::draw3DVector(*viewer, zero, gz, 0, 0, 1, "gz");



    int w = scene_rgb.cols;
    int h = scene_rgb.rows;
    cv::Mat igx = cv::Mat(h, w, CV_8UC1);
    cv::Mat igy = cv::Mat(h, w, CV_8UC1);
    cv::Mat igz = cv::Mat(h, w, CV_8UC1);

    float quantum = parameters->getFloat("quantum");
    for (int i = 0; i < scene_normals->points.size(); i++) {

        Eigen::Vector3f np;
        np << scene_normals->points[i].normal_x,
                scene_normals->points[i].normal_y,
                scene_normals->points[i].normal_z
                ;


        float mag_x = (np.dot(gx) + 1.0f) / 2.0f;
        float mag_y = (np.dot(-gy) + 1.0f) / 2.0f;
        float mag_z = (np.dot(gz) + 1.0f) / 2.0f;

        igx.at<uchar>(i / w, i % w) = floor(mag_x * 255 / quantum) * quantum + quantum / 2.0f;
        igy.at<uchar>(i / w, i % w) = floor(mag_y * 255 / quantum) * quantum + quantum / 2.0f;
        igz.at<uchar>(i / w, i % w) = floor(mag_z * 255 / quantum) * quantum + quantum / 2.0f;

    }

    cv::namedWindow("rgb", cv::WINDOW_NORMAL);
    cv::namedWindow("igx", cv::WINDOW_NORMAL);
    cv::namedWindow("igy", cv::WINDOW_NORMAL);
    cv::namedWindow("igz", cv::WINDOW_NORMAL);

    cv::imshow("rgb", scene_rgb);
    cv::imshow("igx", igx);
    cv::imshow("igy", igy);
    cv::imshow("igz", igz);


    cv::Mat igz_edges = cv::Mat(h, w, CV_8UC1);
    cv::Mat rgb_edges = cv::Mat(h, w, CV_8UC1);
    /// Reduce noise with a kernel 3x3
    cv::blur(igz, igz, cv::Size(3, 3));

    float thresh = parameters->getFloat("edge_th");

    /// Canny detector
    cv::Canny(igz, igz_edges, thresh, thresh * 2, 3);
    cv::Canny(scene_rgb, rgb_edges, thresh, thresh * 2, 3);
    cv::imshow("igz_edges", igz_edges);
    cv::imshow("rgb_edges", rgb_edges);


    std::vector<cv::Vec3f> circles;
    cv::Mat scene_gray;
    cv::cvtColor(scene_rgb, scene_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(scene_gray, scene_gray, cv::Size(9, 9), 2, 2);
    cv::Mat circles_img = scene_gray.clone();
    /// Apply the Hough Transform to find the circles
    cv::HoughCircles(scene_gray, circles, CV_HOUGH_GRADIENT, 1, scene_gray.rows / 8, 200, 100, 0, 0);

    std::cout << "Circles: " << circles.size() << std::endl;
    /// Draw the circles detected
    for (size_t i = 0; i < circles.size(); i++) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        cv::circle(circles_img, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // circle outline
        cv::circle(circles_img, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
    }
    cv::imshow("circles_img", circles_img);

    viewer->addPointCloud(cloud, "scene");
    viewer->addPointCloudNormals<PointType, NormalType>(cloud, gravity_normals_cloud);

    // Creating the KdTree object for the search method of the extraction
    //    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    //    tree->setInputCloud(cloud_filtered);
    //
    //    std::vector<pcl::PointIndices> cluster_indices;
    //    pcl::EuclideanClusterExtraction<PointType> ec;
    //    ec.setClusterTolerance(0.02); // 2cm
    //    ec.setMinClusterSize(100);
    //    ec.setMaxClusterSize(25000);
    //    ec.setSearchMethod(tree);
    //    ec.setInputCloud(cloud_filtered);
    //    ec.extract(cluster_indices);



    while (!viewer->wasStopped()) {
        viewer->spinOnce();
        cv::waitKey(10);
    }

    return (0);
}