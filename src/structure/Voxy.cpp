/* 
 * File:   Voxy.cpp
 * Author: daniele
 * 
 * Created on July 23, 2015, 6:38 PM
 */

#include "Voxy.h"
namespace visy {

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

        std::fill(this->voxel_data, this->voxel_data + (int) this->edge_full_size, 2.0);
        std::fill(this->voxel_data_pin, this->voxel_data_pin + (int) this->edge_full_size, false);
    }

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

    double Voxy::truncatedDistance(Eigen::Vector3f& p1, Eigen::Vector3f& p2) {
        double td = (p1 - p2).norm();
        td = td / this->distance_sigma;
        return td <= 1.0 ? td : 1.0;
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

        while (!boundary_left) {
            boundary_left = !pointToIndex(cursor_left, index_left);
            if (!boundary_left) {
                if (!this->voxel_data_pin[index_left]) {
                    this->voxel_data_pin[index_left] = true;
                    this->voxel_data[index_left] = truncatedDistance(point, cursor_left);
                }
            }
            cursor_left = cursor_left - ray_dir * this->voxel_size;
        }

        while (!boundary_right) {
            boundary_right = !pointToIndex(cursor_right, index_right);
            if (!boundary_right) {
                if (!this->voxel_data_pin[index_right]) {
                    this->voxel_data_pin[index_right] = true;
                    this->voxel_data[index_right] = -truncatedDistance(point, cursor_right);
                }
            }
            cursor_right = cursor_right + ray_dir * this->voxel_size;
        }
    }

    void Voxy::addPointCloud(pcl::PointCloud<PointType>::Ptr cloud, Eigen::Vector3f& pov) {
//        std::fill(this->voxel_data_pin, this->voxel_data_pin + (int) this->edge_full_size, false);
        for (int i = 0; i < cloud->points.size(); i++) {
            if (i % 5 != 0)continue;
            std::cout << "Perc: " << ((double) i / (double) cloud->points.size())*100.0 << std::endl;
            Eigen::Vector3f point(
                    cloud->points[i].x,
                    cloud->points[i].y,
                    cloud->points[i].z
                    );
            this->addPoint(point, pov);

        }
    }
}

