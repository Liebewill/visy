/* 
 * File:   Bold3DDescriptor.h
 * Author: daniele
 *
 * Created on 10 marzo 2015, 18.47
 */

#ifndef BOLD3DDESCRIPTOR_H
#define	BOLD3DDESCRIPTOR_H

#include "Descriptor.h"



namespace visy {
    namespace descriptors {

        class Bold3DDescriptorMultiBunch : public Descriptor {
        public:
            Bold3DDescriptorMultiBunch(int n_bins,std::vector<int>& sizes);
            virtual ~Bold3DDescriptorMultiBunch();
            virtual void describe(cv::Mat& source,pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor);
            virtual void pairKeyPoint3D(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results);
            virtual std::string buildNameImpl();
        protected:
            std::vector<int> sizes;
            int n_bins;
        };
    }
}
#endif	/* BOLD3DDESCRIPTOR_H */

