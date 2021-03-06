/* 
 * File:   KeyPoint3D.cpp
 * Author: daniele
 * 
 * Created on 9 marzo 2015, 18.03
 */

#include "KeyPoint3D.h"

namespace visy {
    namespace extractors {

        int KeyPoint3D::STATIC_COUNTER = 0;

        KeyPoint3D::KeyPoint3D(cv::Vec4f line, pcl::PointCloud<PointType>::Ptr cloud) : cv::KeyPoint() {

            cv::Point p0 = cv::Point2i(line[0], line[1]);
            cv::Point p1 = cv::Point2i(line[2], line[3]);

            this->pt = cv::Point((line[2] + line[0]) / 2, (line[3] + line[1]) / 2);
            this->pt1 = p0;
            this->pt2 = p1;

            cv::Point vdir = p1 - p0;
            this->angle = atan2(vdir.y, vdir.x)*(180 / M_PI);
            this->octave = 0;
            this->size = sqrt(pow(vdir.x, 2) + pow(vdir.y, 2));
            this->class_id = -1;


            float x, y, z;

            int point0_3d_index = (int) (p0.x) + (int) (p0.y * cloud->width);
            int point1_3d_index = (int) (p1.x) + (int) (p1.y * cloud->width);
            
            if (point0_3d_index < cloud->points.size() && point1_3d_index < cloud->points.size()) {
                x = cloud->points[point0_3d_index].x;
                y = cloud->points[point0_3d_index].y;
                z = cloud->points[point0_3d_index].z;
                cv::Point3f p03D;
                if (pcl::isFinite(cloud->points[point0_3d_index]))
                    p03D = cv::Point3f(x, y, z);
                else
                    p03D = cv::Point3f(0, 0, 0);


                x = cloud->points[point1_3d_index].x;
                y = cloud->points[point1_3d_index].y;
                z = cloud->points[point1_3d_index].z;
                cv::Point3f p13D = cv::Point3f(x, y, z);
                if (pcl::isFinite(cloud->points[point1_3d_index]))
                    p13D = cv::Point3f(x, y, z);
                else
                    p13D = cv::Point3f(0, 0, 0);

                float nx = (p13D.x + p03D.x) / 2.0f;
                float ny = (p13D.y + p03D.y) / 2.0f;
                float nz = (p13D.z + p03D.z) / 2.0f;

                this->pt3D = cv::Point3f(nx, ny, nz);

                float dx = (p13D.x - p03D.x);
                float dy = (p13D.y - p03D.y);
                float dz = (p13D.z - p03D.z);

                this->direction_x = cv::Point3f(dx, dy, dz);
                this->pt3D_1 = p03D;
                this->pt3D_2 = p13D;

                this->type = KEYPOINT3D_TYPE_UNKNOWN;
            } else {
                this->pt3D = cv::Point3f(0, 0, 0);
                this->type == KEYPOINT3D_TYPE_INVALID;
            }
        }

        KeyPoint3D::KeyPoint3D(cv::KeyPoint kp, pcl::PointCloud<PointType>::Ptr cloud) {
            this->pt = kp.pt;
            this->angle = kp.angle;
            this->octave = kp.octave;
            this->size = kp.size;
            this->class_id = kp.class_id;

            cv::Point2f vdir = cv::Point2f(cos(this->angle * M_PI / 180.0f) * kp.size / 2.0f, sin(this->angle * M_PI / 180.0f) * kp.size / 2.0f);
            this->pt1 = kp.pt + vdir;
            this->pt2 = kp.pt - vdir;

            float x, y, z;

            int point0_3d_index = (int) (this->pt1.x) + (int) (this->pt1.y * cloud->width);
            int point1_3d_index = (int) (this->pt2.x) + (int) (this->pt2.y * cloud->width);

            if (point0_3d_index < cloud->points.size() && point1_3d_index < cloud->points.size()) {
                x = cloud->points[point0_3d_index].x;
                y = cloud->points[point0_3d_index].y;
                z = cloud->points[point0_3d_index].z;
                cv::Point3f p03D;
                if (pcl::isFinite(cloud->points[point0_3d_index]))
                    p03D = cv::Point3f(x, y, z);
                else
                    p03D = cv::Point3f(0, 0, 0);


                x = cloud->points[point1_3d_index].x;
                y = cloud->points[point1_3d_index].y;
                z = cloud->points[point1_3d_index].z;

                cv::Point3f p13D = cv::Point3f(x, y, z);
                if (pcl::isFinite(cloud->points[point1_3d_index]))
                    p13D = cv::Point3f(x, y, z);
                else
                    p13D = cv::Point3f(0, 0, 0);

                float nx = (p13D.x + p03D.x) / 2.0f;
                float ny = (p13D.y + p03D.y) / 2.0f;
                float nz = (p13D.z + p03D.z) / 2.0f;

                this->pt3D = cv::Point3f(nx, ny, nz);

                float dx = (p13D.x - p03D.x);
                float dy = (p13D.y - p03D.y);
                float dz = (p13D.z - p03D.z);

                this->direction_x = cv::Point3f(dx, dy, dz);
                this->pt3D_1 = p03D;
                this->pt3D_2 = p13D;
                this->type = KEYPOINT3D_TYPE_UNKNOWN;
            } else {
                this->pt3D = cv::Point3f(0, 0, 0);
                this->type == KEYPOINT3D_TYPE_INVALID;
            }

        }

        KeyPoint3D::KeyPoint3D() {
            this->type = KEYPOINT3D_TYPE_UNKNOWN;
        }

        KeyPoint3D::~KeyPoint3D() {
        }

        /**
         * 
         * @param out
         * @param keypoints
         * @param color
         * @param tick
         */
        void
        KeyPoint3D::draw3DKeyPoints(cv::Mat& out, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Scalar color, float tick, bool force_color) {
            for (int i = 0; i < keypoints.size(); i++) {
                visy::extractors::KeyPoint3D kp = keypoints[i];

                cv::Point2f p1 = kp.pt - cv::Point2f(cos(kp.angle * M_PI / 180.0f) * kp.size / 2.0f, sin(kp.angle * M_PI / 180.0f) * kp.size / 2.0f);
                cv::Point2f p2 = kp.pt - cv::Point2f(-cos(kp.angle * M_PI / 180.0f) * kp.size / 2.0f, -sin(kp.angle * M_PI / 180.0f) * kp.size / 2.0f);

                if (!force_color) {
                    if (kp.type == KeyPoint3D::KEYPOINT3D_TYPE_EDGE_TEXTURE) {
                        color = cv::Scalar(0, 0, 255);
                    } else if (kp.type == KeyPoint3D::KEYPOINT3D_TYPE_EDGE_SURFACE) {
                        color = cv::Scalar(0, 255, 0);
                    } else if (kp.type == KeyPoint3D::KEYPOINT3D_TYPE_EDGE_OCCLUSION) {
                        color = cv::Scalar(255, 0, 0);
                    }
                }
                cv::line(out, p1, p2, color, tick);
                cv::circle(out, keypoints[i].pt, 3.0f, color, tick);
            }

        }

        /**
         * 
         * @param viewer
         * @param keypoints
         * @param color
         */
        void
        KeyPoint3D::draw3DKeyPoints3D(pcl::visualization::PCLVisualizer& viewer, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Scalar color, std::string name, bool simple) {

            pcl::PointCloud<PointType>::Ptr keypoint_cloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<NormalType>::Ptr keypoint_normals(new pcl::PointCloud<NormalType>());

            std::stringstream ss;
            for (int i = 0; i < keypoints.size(); i++) {

                KeyPoint3D kp = keypoints[i];

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
                visy::tools::draw3DVector(viewer, vstart, vend, 1.0f, 0, 0, ss.str());

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
        }

        KeyPoint3D
        KeyPoint3D::clone() {

            KeyPoint3D kp3d;
            kp3d.angle = this->angle;
            kp3d.class_id = this->class_id;
            kp3d.direction_x = this->direction_x;
            kp3d.direction_y = this->direction_y;
            kp3d.direction_z = this->direction_z;
            kp3d.octave = this->octave;
            kp3d.pt = this->pt;
            kp3d.pt1 = this->pt1;
            kp3d.pt2 = this->pt2;
            kp3d.pt3D = this->pt3D;
            kp3d.pt3D_1 = this->pt3D_1;
            kp3d.pt3D_2 = this->pt3D_2;
            kp3d.response = this->response;
            kp3d.size = this->size;
            kp3d.type = this->type;

            return kp3d;
        }

        KeyPoint3D
        KeyPoint3D::cloneTranslated(pcl::PointCloud<PointType>::Ptr cloud, cv::Point2f& translation_2d) {
            cv::KeyPoint kp;

            cv::Point2i trans(round(translation_2d.x), round(translation_2d.y));
            kp.pt = this->pt + translation_2d;
            kp.angle = this->angle;
            kp.class_id = this->class_id;
            kp.octave = this->octave;
            kp.response = this->response;
            kp.size = this->size;

            return KeyPoint3D(kp, cloud);
        }

        void
        KeyPoint3D::transformKeyPoint3D(KeyPoint3D& in, KeyPoint3D& out, Eigen::Matrix4f& transform) {
            visy::tools::transformVector(in.pt3D, out.pt3D, transform);
            visy::tools::transformVector(in.pt3D_1, out.pt3D_1, transform);
            visy::tools::transformVector(in.pt3D_2, out.pt3D_2, transform);
            Eigen::Matrix4f rotation = visy::tools::rotationMatrixFromTransformationMatrix(transform);
            visy::tools::transformVector(in.direction_x, out.direction_x, rotation);
            visy::tools::transformVector(in.direction_y, out.direction_y, rotation);
            visy::tools::transformVector(in.direction_z, out.direction_z, rotation);
        }

        void
        KeyPoint3D::transformKeyPoint3Ds(std::vector<KeyPoint3D>& keypoint_in, std::vector<KeyPoint3D>& keypoints_out, Eigen::Matrix4f& transform) {
            keypoints_out.clear();
            for (int i = 0; i < keypoint_in.size(); i++) {
                KeyPoint3D kp3d = keypoint_in[i].clone();
                transformKeyPoint3D(keypoint_in[i], kp3d, transform);
                keypoints_out.push_back(kp3d);
            }
        }



    }
}