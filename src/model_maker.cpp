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
        vstart << kp.pt3D.x - kp.direction_x.x / 2.0f, kp.pt3D.y - kp.direction_x.y / 2.0f, kp.pt3D.z - kp.direction_x.z / 2.0f;
        vend << kp.pt3D.x + kp.direction_x.x / 2.0f, kp.pt3D.y + kp.direction_x.y / 2.0f, kp.pt3D.z + kp.direction_x.z / 2.0f;


        ss.str("");
        ss << name << "_kp_x" << i;
        visy::tools::draw3DVector(viewer, vstart, vend, color[2] / 255.0f, color[1] / 255.0f, color[0] / 255.0f, ss.str());

        if (!simple) {
            float scale = 0.01f;
            vstart << kp.pt3D.x, kp.pt3D.y, kp.pt3D.z;
            vend << kp.pt3D.x + kp.direction_z.x *scale, kp.pt3D.y + kp.direction_z.y *scale, kp.pt3D.z + kp.direction_z.z *scale;

            ss.str("");
            ss << name << "_kp_z" << i;
            visy::tools::draw3DVector(viewer, vstart, vend, 0.0f, 0, 1.0f, ss.str());


            scale = 1.0f;
            vstart << kp.pt3D.x, kp.pt3D.y, kp.pt3D.z;
            vend << kp.pt3D.x + kp.direction_y.x *scale, kp.pt3D.y + kp.direction_y.y *scale, kp.pt3D.z + kp.direction_y.z *scale;

            ss.str("");
            ss << name << "_kp_y" << i;
            visy::tools::draw3DVector(viewer, vstart, vend, 0.0f, 1.0f, 0.0f, ss.str());
        }

    }

    visy::tools::displayCloud(viewer, keypoint_cloud, color[2], color[1], color[0], 5.0f, name);

    /**2D*/
    for (int i = 0; i < keypoints.size(); i++) {
        visy::extractors::KeyPoint3D kp = keypoints[i];

        cv::Point2f p1 = kp.pt - cv::Point2f(cos(kp.angle * M_PI / 180.0f) * kp.size / 2.0f, sin(kp.angle * M_PI / 180.0f) * kp.size / 2.0f);
        cv::Point2f p2 = kp.pt - cv::Point2f(-cos(kp.angle * M_PI / 180.0f) * kp.size / 2.0f, -sin(kp.angle * M_PI / 180.0f) * kp.size / 2.0f);

        if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_TEXTURE) {
            color = cv::Scalar(0, 0, 255);
        } else if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_SURFACE) {
            color = cv::Scalar(0, 255, 0);
        } else if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION) {
            color = cv::Scalar(255, 0, 0);
        } else if (kp.type == visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION_EXT) {
            color = cv::Scalar(0, 0, 0);
        }

        float tick = 1.0f;

        if (!parallels) {
            if (scene_keypoint_selected_index >= 0 && scene_keypoint_selected_index == i) {

                tick = 2.0f;
                //color = cv::Scalar(255, 255, 0);

                float angle = scene_keypoints[i].angle * M_PI / 180.0f;
                std::cout << "Angle: " << angle << std::endl;
                cv::Point2f orientation(cos(angle), sin(angle));
                std::cout << "Or: " << orientation << std::endl;
                cv::Point2f perp(-orientation.y, orientation.x);
                perp.x = perp.x / cv::norm(perp);
                perp.y = perp.y / cv::norm(perp);

                float pd = 1.0f;

                scene_keypoints_parallels.clear();
                scene_keypoints_parallels_all.clear();
                for (int pe = -5; pe <= 5; pe++) {
                    if (pe == 0)continue;

                    cv::Point2f distf = perp * pd * (pe + 1);
                    visy::extractors::KeyPoint3D kp3d = scene_keypoints[i].cloneTranslated(scene_cloud, distf);
                    ext->checkKeyPoint3DType(scene_rgb, scene_cloud, kp3d);
                    scene_keypoints_parallels_all.push_back(kp3d);
                }

                /* BEST KEYPOINT*/
                std::sort(scene_keypoints_parallels_all.begin(), scene_keypoints_parallels_all.end(), KeyPoint3DZOrderer());

                /* FILTERING EXT EDGES*/
                for (int sp = 0; sp < scene_keypoints_parallels_all.size(); sp++) {
                    if (scene_keypoints_parallels_all[sp].type != visy::extractors::KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION_EXT) {
                        scene_keypoints_parallels.push_back(scene_keypoints_parallels_all[sp]);
                    }
                }

                cv::Point3f last_distance_v;
                float last_distance = -1.0f;
                int sel_index = -1;
                for (int sp = 0; sp < scene_keypoints_parallels.size(); sp++) {
                    if (sp == 0)continue;
                    last_distance_v = scene_keypoints_parallels[sp].pt3D - scene_keypoints_parallels[sp - 1].pt3D;
                    last_distance = cv::norm(last_distance_v);
                    std::cout << "V" << sp << " " << last_distance << std::endl;
                    if (last_distance > 0.03f) {
                        sel_index = sp - 1;
                        break;
                    }
                }
                if (sel_index < 0) {
                    sel_index = scene_keypoints_parallels.size() - 1;
                }
                scene_keypoints_parallels_best = sel_index;


            }
        }
        cv::line(out, p1, p2, color, tick);
        cv::circle(out, keypoints[i].pt, 3.0f, color, 1.0f);

    }

    if (!parallels && scene_keypoints_parallels.size() > 0) {

        viewer.removeAllShapes();
        draw3DKeyPointsColor(viewer, out_perp, scene_keypoints_parallels, cv::Scalar(255, 255, 0), "kp parallels", true, true);
    }
}

void
redraw() {
    out = scene_rgb.clone();
    out_perp = scene_rgb.clone();
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addPointCloud(scene_cloud, "scene");
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
    parameters->putBool("x");
    parameters->putBool("f");
    parameters->putFloat("f_th");
    parameters->putFloat("quantum");
    parameters->putFloat("edge_th");


    viewer = new pcl::visualization::PCLVisualizer("Bunch Tester Viewer");

    /** DATASET */
    std::cout << "Building Dataset: " << parameters->getString("dataset") << std::endl;
    visy::dataset::WillowDataset dataset(parameters->getString("dataset"));
    visy::dataset::Model model(parameters->getString("model"), 10);



    int set_number = parameters->getInt("set");
    int scene_number = parameters->getInt("scene");
    std::vector<visy::dataset::Annotation> annotations;
    dataset.loadScene(set_number, scene_number, scene_cloud, scene_rgb);
    dataset.loadAnnotiationsFromSceneFile(model.name, set_number, scene_number, annotations);

    //    // Create the filtering object
    //    pcl::VoxelGrid<PointType> sor;
    //    sor.setInputCloud(scene_cloud);
    //    float leaf = parameters->getFloat("f_th");
    //    sor.setLeafSize(leaf,leaf,leaf);
    //    sor.filter(*scene_cloud_filtered);
    //
    //    viewer->addPointCloud(scene_cloud_filtered, "scene");
    //
    //    while (!viewer->wasStopped()) {
    //        cv::waitKey(100);
    //        viewer->spinOnce();
    //    }
    //
    //    if (true)
    //        return 0;

    /** SCENE NORMALS*/
    // estimate normals
    pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>);

    pcl::IntegralImageNormalEstimation<PointType, NormalType> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(scene_cloud);
    ne.compute(*scene_normals);


    //BUILDS KDTREE SPACE SEARCH
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(scene_cloud);

    std::vector<int> found_indices;
    std::vector<float> indices_distances;

    int w = scene_cloud->width;
    int h = scene_cloud->height;
    out = cv::Mat(h, w, CV_8UC1);
    out2 = cv::Mat(h, w, CV_8UC1);
    out3 = cv::Mat(h, w, CV_8UC1);
    outtg = cv::Mat(h, w, CV_8UC1);
    float quantum = parameters->getFloat("quantum");
    for (int i = 0; i < scene_normals->points.size(); i++) {

        cv::Point3f np(
                scene_normals->points[i].normal_x,
                scene_normals->points[i].normal_y,
                scene_normals->points[i].normal_z
                );

        cv::Point3f z(1, 1, 1);
        z = z * (1.0f / cv::norm(z));
        cv::Point3f x(1, 0, 0);
        cv::Point3f y(0, 1, 0);
        float mag_x = (np.dot(x) + 1.0f) / 2.0f;
        float mag_y = (np.dot(y) + 1.0f) / 2.0f;
        float mag_z = (np.dot(z) + 1.0f) / 2.0f;
        float mag_tg = (np.cross(y).dot(x) + 1.0f) / 2.0f;

        out.at<uchar>(i / w, i % w) = floor(mag_x * 255 / quantum) * quantum + quantum / 2.0f;
        out2.at<uchar>(i / w, i % w) = floor(mag_y * 255 / quantum) * quantum + quantum / 2.0f;
        out3.at<uchar>(i / w, i % w) = floor(mag_z * 255 / quantum) * quantum + quantum / 2.0f;
        outtg.at<uchar>(i / w, i % w) = floor(mag_tg * 255 / quantum) * quantum + quantum / 2.0f;
        //        out.at<cv::Vec3i>(i/w,i%w)[1] = 0;//scene_normals->points[i].normal_y ;//* 255;
        //        out.at<cv::Vec3i>(i/w,i%w)[2] = 0;//scene_normals->points[i].normal_z ;//* 255;
    }

    cv::medianBlur(out, out, 5);
    cv::medianBlur(out2, out2, 5);
    cv::medianBlur(out3, out3, 5);
    cv::medianBlur(outtg, outtg, 5);

    cv::namedWindow("out", cv::WINDOW_NORMAL);
    cv::namedWindow("out2", cv::WINDOW_NORMAL);
    cv::namedWindow("out3", cv::WINDOW_NORMAL);
    cv::namedWindow("lout", cv::WINDOW_NORMAL);
    cv::namedWindow("lout2", cv::WINDOW_NORMAL);
    cv::namedWindow("lout3", cv::WINDOW_NORMAL);
    cv::namedWindow("lout_sum", cv::WINDOW_NORMAL);
    cv::namedWindow("lout_sum_lsd", cv::WINDOW_NORMAL);
    cv::imshow("out", out);
    cv::imshow("out2", out2);
    cv::imshow("out3", out3);
    cv::imshow("outtg", outtg);


    cv::Mat out_lines;
    cv::Mat out_lines2;
    cv::Mat out_lines3;
    cv::Mat out_lines4;
    cv::Mat out_lines_sum;
    cv::Mat out_lines_sum_lsd;

    float thresh = parameters->getFloat("edge_th");
    cv::Canny(out, out_lines, thresh, thresh * 2, 3);
    cv::Canny(out2, out_lines2, thresh, thresh * 2, 3);
    cv::Canny(out3, out_lines3, thresh, thresh * 2, 3);
    cv::Canny(outtg, out_lines4, thresh, thresh * 2, 3);
    //    cv::cvtColor(out_lines,out_lines,CV_GRAY2BGR);
    //    for(int i =0; i < lines.size(); i++){
    //        cv::Vec4f line = lines[i];
    //        cv::line(out_lines,cv::Point(line[0],line[1]),cv::Point(line[2],line[3]),cv::Scalar(0,0,255));
    //    }
    cv::imshow("lout", out_lines);
    cv::imshow("lout2", out_lines2);
    cv::imshow("lout3", out_lines3);
    cv::imshow("louttg", out_lines4);

    out_lines_sum = cv::Mat(480,640,CV_8SC1);
    float fiv = 4.0f;
    out_lines_sum = out/fiv + out2/fiv + out3/fiv+ outtg/fiv;
    for (int i = 0; i < out_lines_sum.rows; i++) {
        for (int j = 0; j < out_lines_sum.cols; j++) {
//            uchar mean = 0;
//            mean += out_lines.at<uchar>(i,j) ;
//            mean += out_lines2.at<uchar>(i,j) ;
//            mean += out_lines3.at<uchar>(i,j) ;
//            mean += out_lines4.at<uchar>(i,j) ;
//            mean = mean / fiv;
           if(out_lines_sum.at<uchar>(i,j) <128){
//               out_lines_sum.at<uchar>(i,j) = 0;
           }
        }
    }
//    out_lines_sum = out_lines / fiv + out_lines2 / fiv + out_lines3 / fiv + out_lines4 / fiv;
    cv::imshow("lout_sum", out_lines_sum);

    std::vector<cv::Vec4f> lsd_lines;
    out_lines_sum_lsd = scene_rgb.clone();
    //    cv::cvtColor(out_lines_sum_lsd, out_lines_sum_lsd, CV_GRAY2BGR);
    edgeDetectionLSDCustom(out_lines_sum, lsd_lines);
    for (int i = 0; i < lsd_lines.size(); i++) {
        cv::Vec4f line = lsd_lines[i];
        cv::line(out_lines_sum_lsd, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2);
    }
    //    std::vector<std::vector<cv::Point> > contours;
    //    std::vector<cv::Vec4i> hierarchy;
    //
    //    /// Detect edges using canny
    //    /// Find contours
    //    cv::findContours(out_lines_sum, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    //    cv::RNG rng(12345);
    //    /// Draw contours
    //    cv::Mat drawing = cv::Mat::zeros(out_lines_sum.size(), CV_8UC3);
    //    for (int i = 0; i < contours.size(); i++) {
    //        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    //        cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
    //    }
    cv::imshow("lout_sum_lsd", out_lines_sum_lsd);

    cv::Mat al = out_lines_sum_lsd.clone();
    cv::cvtColor(al, al, cv::COLOR_BGR2HSV);

    cv::imshow("lout_sum_lsd_hsv", al);

    //    cv::setMouseCallback("out", CallBackFunc, NULL);
    //    cv::namedWindow("out_perp", cv::WINDOW_NORMAL);
    //    cv::setMouseCallback("out_perp", CallBackFunc, NULL);

    viewer->addPointCloud(scene_cloud, "scene");
    viewer->addPointCloudNormals<PointType, NormalType>(scene_cloud, scene_normals);

    //    cv::Mat img = out2;
    //    cv::Mat cimg;
    //    cv::medianBlur(img, img, 5);
    //    cv::cvtColor(img, cimg, cv::COLOR_GRAY2BGR);
    //
    //    std::vector<cv::Vec3f> circles;
    //    cv::HoughCircles(img, circles, CV_HOUGH_GRADIENT, 1, 10,
    //                 100, 30, 1, 30 // change the last two parameters
    //                                // (min_radius & max_radius) to detect larger circles
    //                 );
    //    for( size_t i = 0; i < circles.size(); i++ )
    //    {
    //        cv::Vec3i c = circles[i];
    //        cv::circle( cimg, cv::Point(c[0], c[1]), c[2], cv::Scalar(0,0,255), 3, 8);
    //        cv::circle( cimg, cv::Point(c[0], c[1]), 2, cv::Scalar(0,255,0), 3, 8);
    //    }
    //
    //    cv::imshow("detected circles", cimg);

    while (!viewer->wasStopped()) {
        cv::waitKey(100);
        viewer->spinOnce();
    }

    return 1;
}
