/* 
 * File:   Bold3DExtractor.h
 * Author: daniele
 *
 * Created on 9 marzo 2015, 18.14
 */

#ifndef BOLD3DEXTRACTOR_H
#define	BOLD3DEXTRACTOR_H

#include "Extractor.h"


namespace visy {
    namespace extractors {

        class Bold3DExtractor : public Extractor {
        public:
            Bold3DExtractor(float zone_radius = -1.0f, float zone_slice = -1.0f);
            Bold3DExtractor(const Bold3DExtractor& orig);
            virtual ~Bold3DExtractor();

            virtual void extract(cv::Mat& source, pcl::PointCloud<PointType>::Ptr cloud, std::vector<KeyPoint3D>& keypoints, cv::Mat* mask = NULL);
            static void draw3DKeyPointsWithAreas (cv::Mat& out, std::vector<visy::extractors::KeyPoint3D>& keypoints, cv::Scalar color, float tick, float radius, float slice);
        private:
            float zone_radius;
            float zone_slice;

            static void bindArea(KeyPoint3D& kp, cv::Size2i source_size, std::vector<int>& area, float radius, float slice);
            static void bindAreas(KeyPoint3D& kp, cv::Size2i source_size, std::vector<int>& area_left, std::vector<int>& area_right, float radius, float slice);

        };

    }
}
#endif	/* BOLD3DEXTRACTOR_H */

