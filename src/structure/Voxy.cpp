/* 
 * File:   Voxy.cpp
 * Author: daniele
 * 
 * Created on July 23, 2015, 6:38 PM
 */

#include "Voxy.h"
namespace visy {

    int VoxyLabel::LABEL_COUNTER = 0;

    Voxy::Voxy(double edge_size, double edge_meters, double distance_sigma, Eigen::Vector3f& offset) {
        this->edge_size = edge_size;
        this->edge_square_size = edge_size*edge_size;
        this->edge_full_size = edge_size * edge_size*edge_size;
        this->edge_meters = edge_meters;
        this->distance_sigma = distance_sigma;
        this->voxel_size = this->edge_meters / this->edge_size;
        this->offset = offset;

        this->voxel_data = new double[(int) this->edge_full_size];
        this->voxel_data_pin = new bool[(int) this->edge_full_size];
        this->voxel_data_counter = new int[(int) this->edge_full_size];
        this->voxel_labels = new int[(int) this->edge_full_size];

        std::fill(this->voxel_data, this->voxel_data + (int) this->edge_full_size, 0.0);
        std::fill(this->voxel_data_pin, this->voxel_data_pin + (int) this->edge_full_size, false);
        std::fill(this->voxel_data_counter, this->voxel_data_counter + (int) this->edge_full_size, 0);
        std::fill(this->voxel_labels, this->voxel_labels + (int) this->edge_full_size, -1);
    }
    static int LABEL_COUNTER = 0;

    Voxy::~Voxy() {
        delete this->voxel_data;
        delete this->voxel_data_pin;
    }

    bool Voxy::pointToIndex(Eigen::Vector3f& point, int& index, bool reverse) {
        if (!reverse) {
            float ix = floor(point(0) / voxel_size);
            float iy = floor(point(1) / voxel_size);
            float iz = floor(point(2) / voxel_size);
            ix += floor(this->offset(0) / voxel_size);
            iy += floor(this->offset(1) / voxel_size);
            iz += floor(this->offset(2) / voxel_size);
            if (
                    ix < this->edge_size &&
                    ix >= 0 &&
                    iy < this->edge_size &&
                    iy >= 0 &&
                    iz < this->edge_size &&
                    iz >= 0
                    ) {
                index = ix + this->edge_size * iy + this->edge_square_size *iz;
                return true;
            }
            return false;
        } else {

            point(2) = (index / (this->edge_square_size)) * this->voxel_size - offset(2);
            point(1) = ((index % ((int) this->edge_square_size)) / this->edge_size) * this->voxel_size - offset(1);
            point(0) = ((index % ((int) this->edge_square_size)) % (int) this->edge_size) * this->voxel_size - offset(0);
            return true;
        }
    }

    bool Voxy::pointToIndex(PointType& point, int& index, bool reverse) {
        if (!reverse) {
            float ix = floor(point.x / voxel_size);
            float iy = floor(point.y / voxel_size);
            float iz = floor(point.z / voxel_size);
            ix += floor(this->offset(0) / voxel_size);
            iy += floor(this->offset(1) / voxel_size);
            iz += floor(this->offset(2) / voxel_size);
            if (
                    ix < this->edge_size &&
                    ix >= 0 &&
                    iy < this->edge_size &&
                    iy >= 0 &&
                    iz < this->edge_size &&
                    iz >= 0
                    ) {

                index = ix + this->edge_size * iy + this->edge_square_size *iz;
                return true;
            }
            return false;
        } else {

            point.z = (index / (this->edge_square_size)) * this->voxel_size - offset(2);
            point.y = ((index % ((int) this->edge_square_size)) / this->edge_size) * this->voxel_size - offset(1);
            point.x = ((index % ((int) this->edge_square_size)) % (int) this->edge_size) * this->voxel_size - offset(0);
            return true;
        }
    }

    double Voxy::truncatedDistance(Eigen::Vector3f& p1, Eigen::Vector3f& p2) {
        double td = (p1 - p2).norm();
        td = td / this->distance_sigma;
        return td;
    }

    void Voxy::addPoint(Eigen::Vector3f& point, Eigen::Vector3f& pov) {

        Eigen::Vector3f ray = point - pov;
        Eigen::Vector3f ray_dir = ray / ray.norm();

        Eigen::Vector3f cursor_left = point;
        Eigen::Vector3f cursor_right = point;
        int index_left = -1;
        int index_right = -1;
        bool boundary_left = false;
        bool boundary_right = false;
        bool boundary = false;
        double d;

        while (!boundary_left) {
            boundary_left = !pointToIndex(cursor_left, index_left);
            d = truncatedDistance(point, cursor_left);
            if (d > 1.0f) {
                break;
            }
            if (!boundary_left) {
//                if (this->voxel_data_counter[index_left] < this->round_counter) {
                    this->voxel_data_counter[index_left]++;
//                    if(this->voxel_data[index_left] ==0){
//                       this->voxel_data[index_left] = d; 
//                    }else{
//                        this->voxel_data[index_left] = 0.9f*this->voxel_data[index_left]+0.1f*d;
//                    }
                    this->voxel_data[index_left] += d;
//                }
            }
            cursor_left = cursor_left - ray_dir * this->voxel_size;
        }

        while (!boundary_right) {
            boundary_right = !pointToIndex(cursor_right, index_right);
            d = truncatedDistance(point, cursor_right);
            //d = d * d*d;
            if (d > 1.0f) {
                break;
            }
            if (!boundary_right) {
//                if (this->voxel_data_counter[index_right] < this->round_counter) {
                    this->voxel_data_counter[index_right]++;
//                    if(this->voxel_data[index_right] ==0){
//                       this->voxel_data[index_right] = -d; 
//                    }else{
//                        this->voxel_data[index_right] = 0.9f*this->voxel_data[index_right]-0.1f*d;
//                    }
                    this->voxel_data[index_right] += -d;
//                }
            }
            cursor_right = cursor_right + ray_dir * this->voxel_size;
        }
    }

    void Voxy::addPointCloud(pcl::PointCloud<PointType>::Ptr cloud, Eigen::Vector3f& pov) {
        //std::fill(this->voxel_data_pin, this->voxel_data_pin + (int) this->edge_full_size, false);
        this->round_counter++;
        for (int i = 0; i < cloud->points.size(); i++) {
            if (i % 10 != 0)continue; 
            //            int x = i % 640;
            //            int y = i / 640;
            //            if (x > 160 && x < 480 && y > 120 && y < 360) {

            //std::cout << "Perc: " << ((double) i / (double) cloud->points.size())*100.0 << std::endl;
            Eigen::Vector3f point(
                    cloud->points[i].x,
                    cloud->points[i].y,
                    cloud->points[i].z
                    );
            this->addPoint(point, pov);
            //            }

        }
    }

    void Voxy::voxelToCloudZeroCrossing(pcl::PointCloud<PointType>::Ptr& cloud_out) {
        Eigen::Vector3f point;
        for (int i = 0; i < this->edge_full_size; i++) {
            if (this->voxel_data_counter[i] <= 0)continue;

            this->pointToIndex(point, i, true);
            //        
            PointType p;
            p.x = point(0);
            p.y = point(1);
            p.z = point(2);


            int n1 = i + 1;
            int n2 = i + this->edge_size;
            int n3 = i + this->edge_square_size;

            bool test = true;
            if (n1 < this->edge_full_size && n2 < this->edge_full_size && n3 < this->edge_full_size) {
                if (this->voxel_data_counter[n1] <= 0)continue;
                if (this->voxel_data_counter[n2] <= 0)continue;
                if (this->voxel_data_counter[n3] <= 0)continue;

                if ((this->voxel_data[i] >= 0 && this->voxel_data[n1] < 0) || (this->voxel_data[i] < 0 && this->voxel_data[n1] >= 0)) {
                    test = false;
                }
                if ((this->voxel_data[i] >= 0 && this->voxel_data[n2] < 0) || (this->voxel_data[i] < 0 && this->voxel_data[n2] >= 0)) {
                    test = false;
                }
                if ((this->voxel_data[i] >= 0 && this->voxel_data[n3] < 0) || (this->voxel_data[i] < 0 && this->voxel_data[n3] >= 0)) {
                    test = false;
                }

            }
            
            if (!test) {
                p.r = 255;
                p.g = 255;
                p.b = 255;
                cloud_out->points.push_back(p);
            }

            //                    if (abs(this->voxel_data[i]) <= 1.0)
            //                        if (this->voxel_data[i] >= 0) {
            //                            p.r = (255.0 - this->voxel_data[i]*255.0);
            //                            p.g = 0;
            //                            p.b = 0;
            //                            cloud_out->points.push_back(p);
            //                        } else {
            //                            p.r = 0;
            //                            p.g = (255.0 + this->voxel_data[i]*255.0);
            //                            p.b = 0;
            //                            cloud_out->points.push_back(p);
            //                        }
        }
    }

    int Voxy::addCluster(pcl::PointCloud<PointType>::Ptr cluster) {

        int perc_counter = 0;
        int found_label = -1;
        for (int i = 0; i < cluster->points.size(); i++) {
            int index;
            if (pointToIndex(cluster->points[i], index)) {
                int label = this->voxel_labels[index];
                if (label>-1) {
                    found_label = label;
                    perc_counter++;
                }
            }
        }

        double perc = (double) perc_counter / (double) cluster->points.size();
        if (found_label == -1 ) {
            found_label = VoxyLabel::LABEL_COUNTER++;
        }

        for (int i = 0; i < cluster->points.size(); i++) {
            int index;
            if (pointToIndex(cluster->points[i], index)) {
                this->voxel_labels[index] = found_label;
            }
        }
    }
    
    int Voxy::labelsCount(){
        return VoxyLabel::LABEL_COUNTER;
    }
}

