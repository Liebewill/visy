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
#include "Parameters.h"
#include <boost/filesystem.hpp>


using namespace std;
using namespace BoldLib;




visy::Parameters* parameters;

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
  parameters->putString("detector");
  parameters->putString("model");
  parameters->putString("sizes");
  parameters->putInt("set");
  parameters->putInt("scene");
  parameters->putInt("nbin");

  visy::dataset::IrosDataset::init();


  visy::detectors::Detector * detector;

  std::vector<float> sizes = visy::Parameters::parseFloatArray(parameters->getString("sizes"));

  if (parameters->getString("detector") == "BOLD3DM")
  {
    detector = new visy::detectors::Bold3DMDetector(sizes, parameters->getInt("nbin"));
  }
  else if (parameters->getString("detector") == "BOLD3DR")
  {
    detector = new visy::detectors::Bold3DRDetector(sizes, parameters->getInt("nbin"));
  }
  else if (parameters->getString("detector") == "BOLD")
  {
    detector = new visy::detectors::BoldDetector(sizes);
  }


  std::cout << detector->buildName() << std::endl;

  std::vector<visy::extractors::KeyPoint3D> model_keypoints;
  cv::Mat model_descriptor;
  pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
  Eigen::Matrix4f model_pose;

  visy::dataset::Model model = visy::dataset::IrosDataset::findModelByName(parameters->getString("model"));

  visy::dataset::IrosDataset dataset;
  dataset.fetchFullModel(model.name, model.n_views, model_keypoints, model_descriptor, model_cloud,model_pose, detector);
  
  
  dataset.saveDescription(model.name, model_keypoints, model_descriptor,model_pose, detector->buildName());



  return 1;
}
