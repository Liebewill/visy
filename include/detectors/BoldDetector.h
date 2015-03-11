/* 
 * File:   BoldDetector.h
 * Author: daniele
 *
 * Created on 11 marzo 2015, 18.05
 */

#ifndef BOLDDETECTOR_H
#define	BOLDDETECTOR_H

#include "Detector.h"


namespace visy {
    namespace detectors {

        class BoldDetector : public Detector {
        public:
            BoldDetector(std::vector<int>& sizes, int n_bins = 12, int scale_levels = 3, float scale_space_factor = 1.6f);
            virtual ~BoldDetector();
            virtual void detect(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Mat& descriptor, cv::Mat* mask = NULL);
            virtual std::string buildNameImpl();
        private:
            BoldLib::BOLD bold;
        };
    }
}
#endif	/* BOLDDETECTOR_H */
// bold.setK(sizes);
//    bold.setNumAngularBins(12);
//    bold.setScaleSpaceLevels(3);
//    bold.setScaleSpaceScaleFactor(1.6f);
//    bold.setHoughThr(4);
//    bold.setHoughBinSize(16);
