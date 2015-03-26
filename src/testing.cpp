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
#include <pcl/common/centroid.h>
#include <pcl/common/intersections.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/centroid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>


#include <sstream>
#include <string>
#include <fstream>

#include <tools.h>
#include <opencv2/core/mat.hpp>

#include "Extractor.h"
#include "Bold3DExtractor.h"
#include "IrosDataset.h"
#include "extractors/extrators_utils.h"
#include "Descriptor.h"
#include "Bold3DDescriptorMultiBunch.h"
#include "Detector.h"
#include "Bold3DMDetector.h"
#include "Bold3DRDetector.h"
#include "BoldDetector.h"
#include "detectors/detectors_utils.h"
#include "Parameters.h"

using namespace std;
using namespace BoldLib;


/*
LineSegmentDetection( int * n_out,
double * img, int X, int Y,
double scale, double sigma_scale, double quant,
double ang_th, double log_eps, double density_th,
int n_bins,
int ** reg_img, int * reg_x, int * reg_y );
 */

visy::Parameters* parameters;
pcl::visualization::PCLVisualizer * viewer;

/*
 *
 */
int
main (int argc, char** argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  /** PARAMETERS */
  parameters = new visy::Parameters(argc, argv);
  parameters->putFloat("gc_th");
  parameters->putFloat("gc_size");
  parameters->putString("detector");
  parameters->putString("model");
  parameters->putString("sizes");
  parameters->putInt("set");
  parameters->putInt("scene");
  parameters->putInt("nbin");
  parameters->putInt("occlusion");

  int use_occlusion = false;

  visy::dataset::IrosDataset dataset;
  visy::dataset::IrosDataset::init();
  visy::dataset::Model model = visy::dataset::IrosDataset::findModelByName(parameters->getString("model"));

  std::vector<visy::extractors::KeyPoint3D> scene_keypoints;
  cv::Mat scene_descriptor;
  cv::Mat scene_rgb, scene_rgb_full;
  pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>());
  visy::dataset::IrosDataset::loadScene(parameters->getInt("set"), parameters->getInt("scene"), scene_cloud, scene_rgb);


  //BOLD

  std::vector<cv::Vec4f> lines;
  std::vector<cv::Vec4f> lines_2;
  visy::tools::edgeDetection(scene_rgb, lines, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_LSD);
  visy::tools::edgeDetection(scene_rgb, lines_2, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD);

  cv::Mat out = scene_rgb.clone();
  cv::Mat out_2 = scene_rgb.clone();
  for (int i = 0; i < lines.size(); i++)
  {

    cv::line(out, cv::Point(lines[i].val[0], lines[i].val[1]), cv::Point(lines[i].val[2], lines[i].val[3]), cv::Scalar(0, 0, 255));
  }
  for (int i = 0; i < lines_2.size(); i++)
  {

    cv::line(out_2, cv::Point(lines_2[i].val[0], lines_2[i].val[1]), cv::Point(lines_2[i].val[2], lines_2[i].val[3]), cv::Scalar(0, 0, 255));
  }

  std::cout << "FOUND:" << lines.size() << std::endl;
  std::cout << "FOUND:" << lines_2.size() << std::endl;
  cv::imshow("LSD", out);
  cv::imshow("BOLD_LSD", out_2);
  cv::waitKey(0);

  return 1;
}
