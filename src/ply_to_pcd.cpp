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

#include "commons/Parameters.h"



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
  parameters->putString("source");
  
  
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
  
  std::string filename = parameters->getString("source");// "/home/daniele/work/data/cvlab_dataset/MeshRegistration/Doll/Doll005.ascii.ply";
  
  std::cout << "Loading file: "<<filename<<std::endl;
  
  pcl::io::loadPLYFile(filename,*cloud);
  

  return 1;
}
