/* 
 * File:   Bold3DDescriptorRadiusBunch.h
 * Author: daniele
 *
 * Created on 11 marzo 2015, 16.21
 */

#ifndef BOLD3DDESCRIPTORRADIUSBUNCH_H
#define	BOLD3DDESCRIPTORRADIUSBUNCH_H

#include "Descriptor.h"



namespace visy {
    namespace descriptors {

        class Bold3DDescriptorRadiusBunch : public Descriptor{
        public:
            Bold3DDescriptorRadiusBunch(int n_bins,std::vector<float>& radiuses);
            virtual ~Bold3DDescriptorRadiusBunch();
            virtual void describe(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor);
            virtual void pairKeyPoint3D(visy::extractors::KeyPoint3D& kp1, visy::extractors::KeyPoint3D& kp2, float** results);
            virtual std::string buildNameImpl();
        protected:
            std::vector<float> radiuses;
            int n_bins;

        };
    }
}

#endif	/* BOLD3DDESCRIPTORRADIUSBUNCH_H */

