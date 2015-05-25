/*
 * File:   bunch_tester.cpp
 * Author: daniele
 *
 * Created on 27 novembre 2014, 22.17
 */

#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boldlib.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/integral_image_normal.h>

#include <sstream>
#include <string>
#include <fstream>

#include <tools.h>
#include <opencv2/core/mat.hpp>

#include "Extractor.h"
#include "Bold3DExtractor.h"
#include "extractors/extrators_utils.h"
#include "Detector.h"

#include "detectors/detectors_utils.h"
#include "commons/commons.h"
#include "WillowDataset.h"
#include "PipeLine.h"

using namespace std;
using namespace BoldLib;

visy::Parameters* parameters;

visy::extractors::Bold3DExtractor* ext;

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
pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr scene_cloud_filtered(new pcl::PointCloud<PointType>());

/**VIEWER*/
pcl::visualization::PCLVisualizer * viewer;

cv::Mat out, out2, out3, outtg, out_perp;

struct KeyPoint3DZOrderer {

    inline bool operator()(const visy::extractors::KeyPoint3D& kp1, const visy::extractors::KeyPoint3D& kp2) {
        return kp1.pt3D.z < kp2.pt3D.z;
    }
};

void
draw3DKeyPointsColor(pcl::visualization::PCLVisualizer& viewer, cv::Mat& out, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Scalar color, std::string name, bool simple, bool parallels = false) {

    pcl::PointCloud<PointType>::Ptr keypoint_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<NormalType>::Ptr keypoint_normals(new pcl::PointCloud<NormalType>());

    std::cout << "Drawing: " << keypoints.size() << std::endl;
    std::stringstream ss;
    for (int i = 0; i < keypoints.size(); i++) {
        visy::extractors::KeyPoint3D kp = keypoints[i];
        if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_TEXTURE) {
            color = cv::Scalar(0, 0, 255);
        } else if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_SURFACE) {
            color = cv::Scalar(0, 255, 0);
        } else if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION) {
            color = cv::Scalar(255, 0, 0);
        } else if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION_EXT) {
            color = cv::Scalar(255, 255, 255);
        }

        if (parallels && scene_keypoints_parallels_best == i) {
            color = cv::Scalar(255, 255, 0);
        }


        PointType p;
        p.x = kp.pt3D.x;
        p.y = kp.pt3D.y;
        p.z = kp.pt3D.z;

        keypoint_cloud->points.push_back(p);

        Eigen::Vector3f vstart, vend;

        float scale = 0.01f;
        vstart << kp.pt3D.x, kp.pt3D.y, kp.pt3D.z;
        vend <<
                kp.pt3D.x + kp.reference_frame.x_axis[0] * scale,
                kp.pt3D.y + kp.reference_frame.x_axis[1] * scale,
                kp.pt3D.z + kp.reference_frame.x_axis[2] * scale;


        ss.str("");
        ss << name << "_kp_x" << i;
        visy::tools::draw3DVector(viewer, vstart, vend, 1.0f, 0, 0.0f, ss.str());

        if (!simple) {

            vstart << kp.pt3D.x, kp.pt3D.y, kp.pt3D.z;
            vend << kp.pt3D.x + kp.reference_frame.z_axis[0] * scale, kp.pt3D.y + kp.reference_frame.z_axis[1] * scale, kp.pt3D.z + kp.reference_frame.z_axis[2] * scale;

            ss.str("");
            ss << name << "_kp_z" << i;
            visy::tools::draw3DVector(viewer, vstart, vend, 0.0f, 0, 1.0f, ss.str());


            vstart << kp.pt3D.x, kp.pt3D.y, kp.pt3D.z;
            vend << kp.pt3D.x + kp.reference_frame.y_axis[0] * scale, kp.pt3D.y + kp.reference_frame.y_axis[1] * scale, kp.pt3D.z + kp.reference_frame.y_axis[2] * scale;

            ss.str("");
            ss << name << "_kp_y" << i;
            visy::tools::draw3DVector(viewer, vstart, vend, 0.0f, 1.0f, 0.0f, ss.str());
        }

    }

    visy::tools::displayCloud(viewer, keypoint_cloud, color[2], color[1], color[0], 5.0f, name);


}

void
redraw() {
    out = scene_rgb.clone();
    out_perp = scene_rgb.clone();
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addPointCloud(cloud, "scene");
    draw3DKeyPointsColor(*viewer, out, scene_keypoints, cv::Scalar(0, 255, 0), "scene_kps", true);
    cv::imshow("out", out);
    cv::imshow("out_perp", out_perp);
}

void
CallBackFunc(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        std::cout << "Click " << x << std::endl;
        float min_dist = 1000.0f;
        int min_index = -1;

        for (int i = 0; i < scene_keypoints.size(); i++) {
            cv::Point2f d = scene_keypoints[i].pt - cv::Point2f(x, y);
            float dd = cv::norm(d);
            if (dd < min_dist) {
                min_dist = dd;
                min_index = i;
            }
        }

        if (min_index >= 0) {
            scene_keypoint_selected_index = min_index;
            scene_keypoints_seletected.clear();
            scene_keypoints_seletected.push_back(scene_keypoints[min_index]);
        } else {
            scene_keypoint_selected_index = -1;
        }
        redraw();
    }
}

void
edgeDetectionLSDCustom(cv::Mat& gray, std::vector<cv::Vec4f>& lines) {
    int n;
    int X = gray.cols;
    int Y = gray.rows;
    double* d_img = (double *) malloc(X * Y * sizeof (double));

    for (int x = 0; x < X; x++) {
        for (int y = 0; y < Y; y++) {
            d_img[x + y * X] = gray.at<uchar>(y, x);
        }
    }
    double scale = 0.8;
    double sigma_scale = 0.6;
    double quant = 2.0;
    double ang_th = 40.5;
    double log_eps = 0.0;
    double density_th = 0.7;
    int n_bins = 1024;

    double * edges;
    edges = LineSegmentDetection(&n, d_img, X, Y, scale, sigma_scale, quant, ang_th, log_eps, density_th, n_bins, NULL, NULL, NULL);

    for (int tp = 0; tp < (int) n; tp++) {
        cv::Vec4f line(
                (float) edges[5 * tp],
                (float) edges[5 * tp + 1],
                (float) edges[5 * tp + 2],
                (float) edges[5 * tp + 3]);
        lines.push_back(line);
    }
    delete[] d_img;
    delete[] edges;
}

typedef struct {
    double r; // percent
    double g; // percent
    double b; // percent
} rgb;

typedef struct {
    double h; // angle in degrees
    double s; // percent
    double v; // percent
} hsv;

static hsv rgb2hsv(rgb in);
static rgb hsv2rgb(hsv in);

hsv rgb2hsv(rgb in) {
    hsv out;
    double min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min < in.b ? min : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max > in.b ? max : in.b;

    out.v = max; // v
    delta = max - min;
    if (max > 0.0) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max); // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, v is undefined
        out.s = 0.0;
        out.h = NAN; // its now undefined
        return out;
    }
    if (in.r >= max) // > is bogus, just keeps compilor happy
        out.h = (in.g - in.b) / delta; // between yellow & magenta
    else
        if (in.g >= max)
        out.h = 2.0 + (in.b - in.r) / delta; // between cyan & yellow
    else
        out.h = 4.0 + (in.r - in.g) / delta; // between magenta & cyan

    out.h *= 60.0; // degrees

    if (out.h < 0.0)
        out.h += 360.0;

    return out;
}

rgb hsv2rgb(hsv in) {
    double hh, p, q, t, ff;
    long i;
    rgb out;

    if (in.s <= 0.0) { // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if (hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long) hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch (i) {
        case 0:
            out.r = in.v;
            out.g = t;
            out.b = p;
            break;
        case 1:
            out.r = q;
            out.g = in.v;
            out.b = p;
            break;
        case 2:
            out.r = p;
            out.g = in.v;
            out.b = t;
            break;

        case 3:
            out.r = p;
            out.g = q;
            out.b = in.v;
            break;
        case 4:
            out.r = t;
            out.g = p;
            out.b = in.v;
            break;
        case 5:
        default:
            out.r = in.v;
            out.g = p;
            out.b = q;
            break;
    }
    return out;
}

/*
 *
 */
int
main(int argc, char** argv) {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

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
    parameters->putInt("hough");
    parameters->putFloat("f_th");
    parameters->putFloat("gt_distance");

    /**VIEWER*/
    pcl::visualization::PCLVisualizer * viewer;
    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");


    /** DETECTOR */
    visy::detectors::Detector * detector;
    visy::detectors::Detector * detector_model;

    detector = visy::detectors::utils::buildDetectorFromString(parameters->getString("detector"), parameters, false);
    detector_model = visy::detectors::utils::buildDetectorFromString(parameters->getString("detector"), parameters, true);

    std::cout << "Detector: " << detector->buildName() << std::endl;
    if (parameters->getString("detector") == "BOLD") {
        std::cout << "Descriptor size: " << 144 << std::endl;

    } else {
        if (detector->descriptor != NULL)
            std::cout << "Descriptor size: " << detector->descriptor->dfunction->getDataSize() << std::endl;

    }

    /** DATASET */
    std::cout << "Building Dataset: " << parameters->getString("dataset") << std::endl;
    visy::dataset::WillowDataset dataset(parameters->getString("dataset"));


    /** SCENE */
    std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
    cv::Mat scene_descriptor;
    cv::Mat scene_rgb, scene_rgb_full;
    pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr scene_cloud_filtered(new pcl::PointCloud<PointType>());

    int set_number = parameters->getInt("set");
    int scene_number = parameters->getInt("scene");
    std::vector<visy::dataset::Annotation> annotations;
    dataset.loadScene(set_number, scene_number, scene_cloud, scene_rgb);
    //    dataset.loadAnnotiationsFromSceneFile(model.name, set_number, scene_number, annotations);

    /** DETECTION */
    boost::posix_time::ptime time_start(boost::posix_time::microsec_clock::local_time());
    detector->detect(scene_rgb, scene_cloud, scene_keypoints, scene_descriptor);
    std::cout << "Scene kps: " << scene_keypoints.size() << std::endl;
    boost::posix_time::ptime time_end(boost::posix_time::microsec_clock::local_time());
    boost::posix_time::time_duration duration(time_end - time_start);
    cout << "Detector time: " << duration << '\n';


    cv::Mat out = scene_rgb.clone();

    draw3DKeyPointsColor(*viewer, out, scene_keypoints, cv::Scalar(0, 0, 255), "kps", false);

    viewer->addPointCloud(scene_cloud, "scene");



    while (!viewer->wasStopped()) {
        cv::waitKey(100);
        viewer->spinOnce();
    }

    return 1;
}
