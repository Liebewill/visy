
#include <tools.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <boldlib.h>
#include <bold_p.h>


namespace visy
{
  namespace tools
  {

    /**
     * Distance between point and line
     * @param v Line first point
     * @param w Line second point
     * @param p Target point
     * @return distance
     */
    float
    point2LineDistance (cv::Point v, cv::Point w, cv::Point p)
    {
      const float l2 = pow(cv::norm(w - v), 2);
      if (l2 == 0.0) return cv::norm(v - p);
      cv::Point pv(p - v);
      cv::Point wv(w - v);
      const float t = pv.dot(wv) / l2;

      if (t < 0.0) return cv::norm(v - p);
      else if (t > 1.0) return cv::norm(w - p);
      cv::Point projection = v + t * (w - v);
      return cv::norm(projection - p);
    }

    /**
     * Generic Edge Detection from Grayscale image
     * @param source
     * @param lines
     * @param method
     */
    void
    edgeDetection (cv::Mat& source, std::vector<cv::Vec4f>& lines, int method)
    {

      lines.clear();
      cv::Mat gray;
      if (source.type() != 0)
      {
        cv::cvtColor(source, gray, CV_BGR2GRAY);
      }
      else
      {
        gray = source;
      }
      if (method == VISY_TOOLS_EDGEDETECTION_METHOD_LSD)
      {
        int n;
        int X = gray.cols;
        int Y = gray.rows;
        double* d_img = (double *) malloc(X * Y * sizeof (double));

        for (int x = 0; x < X; x++)
        {
          for (int y = 0; y < Y; y++)
          {
            d_img[x + y * X] = gray.at<uchar>(y, x);
          }
        }
        double scale = 0.8;
        double sigma_scale = 0.6;
        double quant = 2.0;
        double ang_th = 22.5;
        double log_eps = 0.0;
        double density_th = 0.7;
        int n_bins = 1024;

        double * edges;
        edges = LineSegmentDetection(&n, d_img, X, Y, scale, sigma_scale, quant, ang_th, log_eps, density_th, n_bins, NULL, NULL, NULL);

        for (int tp = 0; tp < (int) n; tp++)
        {
          cv::Vec4f line(
                  (float) edges[5 * tp],
                  (float) edges[5 * tp + 1],
                  (float) edges[5 * tp + 2],
                  (float) edges[5 * tp + 3]);
          lines.push_back(line);
        }
      }
    }

    void
    planeCoefficients (pcl::PointCloud<PointType>::Ptr cloud, std::vector<int>& indices, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers,float distance_th )
    {
      pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(distance_th);
      
      pcl::PointCloud<PointType>::Ptr sub_cloud(new pcl::PointCloud<PointType>());
      pcl::copyPointCloud(*cloud, indices, *sub_cloud);
      
      seg.setInputCloud(sub_cloud);
      seg.segment(*inliers, *coefficients);
    }

    /**
     * Retrieves RGB image from CLOUD
     * @param cloud [INPUT] source cloud
     * @param image [OUTPUT] image
     */
    void
    rgbFromCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, cv::Mat& image)
    {
      //            image.create(cloud->height, cloud->width, CV_8UC3);
      image = cv::Mat::zeros(cloud->height, cloud->width, CV_8UC3);

      int width = cloud->width;

      for (int index = 0; index < cloud->size(); index++)
      {
        int x = index % width;
        int y = index / width;
        image.at<cv::Vec3b>(y, x)[0] = cloud->points[index].b;
        image.at<cv::Vec3b>(y, x)[1] = cloud->points[index].g;
        image.at<cv::Vec3b>(y, x)[2] = cloud->points[index].r;
      }
    }

    /**
     * see rgbFromCloud with selected indices
     * @param cloud [INPUT] source cloud
     * @param image [OUTPUT] output image
     * @param indices [INPUT] selected indices
     */
    void
    rgbFromCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, cv::Mat& image, std::vector<int>& indices)
    {
      //            image.create(cloud->height, cloud->width, CV_8UC3);
      image = cv::Mat::zeros(cloud->height, cloud->width, CV_8UC3);
      int width = cloud->width;

      int index = -1;
      for (int i = 0; i < indices.size(); i++)
      {
        index = indices[i];
        int x = index % width;
        int y = index / width;
        image.at<cv::Vec3b>(y, x)[0] = cloud->points[index].b;
        image.at<cv::Vec3b>(y, x)[1] = cloud->points[index].g;
        image.at<cv::Vec3b>(y, x)[2] = cloud->points[index].r;
      }
    }

    /**
     * Converts pcl::PointXYZ to Eigen::Vector3f and conversely
     * @param pt pcl Point
     * @param p Eigen Point
     * @param reverse TRUE if inverse transform
     */
    void
    convertPoint3D (PointType& pt, Eigen::Vector3f& p, bool reverse)
    {
      if (!reverse)
      {
        p[0] = pt.x;
        p[1] = pt.y;
        p[2] = pt.z;
      }
      else
      {
        pt.x = p[0];
        pt.y = p[1];
        pt.z = p[2];
      }
    }

    /**
     * Draws 3D vector
     * @param viewer target viewer
     * @param start start point
     * @param end end point
     * @param r Red
     * @param g Green
     * @param b Blue
     * @param name ID_NAME for viewer (no duplicates)
     */
    void
    draw3DVector (pcl::visualization::PCLVisualizer& viewer, Eigen::Vector3f start, Eigen::Vector3f end, float r, float g, float b, std::string name)
    {
      PointType p_start, p_end;
      visy::tools::convertPoint3D(p_start, start, true);

      Eigen::Vector3f dv;
      dv = end - start;

      p_end.x = p_start.x + dv[0];
      p_end.y = p_start.y + dv[1];
      p_end.z = p_start.z + dv[2];

      viewer.addArrow(p_end, p_start, r, g, b, false, name);
    }

    /**
     * Displays Cloud in viewer
     * @param viewer target viewer
     * @param cloud target cloud
     * @param r Red
     * @param g Green
     * @param b Blue
     * @param size point size
     * @param name ID_NAME for viewer (no duplicates)
     */
    void
    displayCloud (pcl::visualization::PCLVisualizer &viewer, pcl::PointCloud<PointType>::Ptr cloud, int r, int g, int b, int size, std::string name)
    {
      pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_color(cloud, r, g, b);
      viewer.addPointCloud(cloud, cloud_color, name);
      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
    }

    void
    display4DHistogram (pcl::visualization::PCLVisualizer& viewer, std::string name, float* histogram, int size, int& viewport)
    {

      pcl::PointCloud<PointType>::Ptr reference(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr histo4d_back(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr histo4d(new pcl::PointCloud<PointType>());


      PointType pr;
      pr.x = -1;
      pr.y = -1;
      pr.z = -1;
      pr.r = 0;
      pr.g = 255;
      pr.b = 0;
      reference->points.push_back(pr);

      int fullSize = size * size*size;
      float max = 0.0;
      float min = 1000.0f;
      float t = 0.0f;
      for (int i = 0; i < fullSize; i++)
      {
        t = histogram[i];
        max = t > max ? t : max;
        min = t < min ? t : min;

      }

      float ratio = 0.0f;
      int r, g, b;
      for (int x = 0; x < size; x++)
      {
        for (int y = 0; y < size; y++)
        {
          for (int z = 0; z < size; z++)
          {
            int index = x + y * size + z * size * size;
            t = histogram[index];
            ratio = t / max;
            r = 255 * ratio;
            g = 40;
            b = 40;

            PointType p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.r = r;
            p.g = g;
            p.b = b;


            if (ratio > 0.0001f)
            {
              histo4d->points.push_back(p);
            }
            else
            {
              histo4d_back->points.push_back(p);
            }

          }
        }
      }

      std::stringstream ss;
      ss << name << "_histo4d";


      viewer.addPointCloud(histo4d, ss.str(), viewport);
      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 25, ss.str(), viewport);

      ss.str("");
      ss << name << "_histo4d_back";
      viewer.addPointCloud(histo4d_back, ss.str(), viewport);
      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss.str(), viewport);

      ss.str("");
      ss << name << "_reference";
      viewer.addPointCloud(reference, ss.str(), viewport);
      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, ss.str(), viewport);


      Eigen::Vector3f a0;
      Eigen::Vector3f ax;
      Eigen::Vector3f ay;
      Eigen::Vector3f az;

      a0 << -1.0f, -1.0f, -1.0f;
      ax << 1.0f, -1.0f, -1.0f;
      ay << -1.0f, 1.0f, -1.0f;
      az << -1.0f, -1.0f, 1.0f;

      ss.str("");
      ss << name << "_ax";
      draw3DVector(viewer, a0, ax, 1.0f, 0.0f, 0.0f, ss.str());
      ss.str("");
      ss << name << "_ay";
      draw3DVector(viewer, a0, ay, 0.0f, 1.0f, 0.0f, ss.str());
      ss.str("");
      ss << name << "_az";
      draw3DVector(viewer, a0, az, 0.0f, 0.0f, 1.0f, ss.str());
    }

    /**
     * Inverts Transformation Matrix
     * @param t
     * @return
     */
    Eigen::Matrix4f
    invertTransformationMatrix (Eigen::Matrix4f& t)
    {
      Eigen::Matrix3f R = t.block<3, 3>(0, 0);
      Eigen::Vector3f translation = t.block<3, 1>(0, 3);
      Eigen::MatrixXf last_block = t.block<1, 4>(3, 0);

      Eigen::Matrix3f RT = R.transpose();
      Eigen::MatrixXf Temp(3, 4);
      translation = -RT*translation;
      Temp << RT, translation;
      Eigen::Matrix4f t2;
      t2 << Temp, last_block;

      return t2;
    }

    void
    poseError (Eigen::Matrix4f& pose1, Eigen::Matrix4f& pose2, float& rotation_error, float& distance_error)
    {

      Eigen::Matrix3f rot1 = pose1.block<3, 3>(0, 0);
      Eigen::Matrix3f rot2 = pose2.block<3, 3>(0, 0);

      Eigen::Quaternion<float> q1(rot1);
      Eigen::Quaternion<float> q2(rot2);

      q1.normalize();
      q2.normalize();

      Eigen::Vector3f translation1 = pose1.block<3, 1>(0, 3);
      Eigen::Vector3f translation2 = pose2.block<3, 1>(0, 3);

      rotation_error = q1.angularDistance(q2);
      if (rotation_error < 0)rotation_error *= -1;
      distance_error = (translation1 - translation2).norm();

    }

    cv::Scalar
    avgColor (cv::Mat& source, cv::Point2i center, float radius)
    {
      cv::Mat mask = cv::Mat::zeros(source.rows, source.cols, CV_8U);
      cv::circle(mask, center, radius, cv::Scalar(255, 255, 255), -1);

      return cv::mean(source, mask);

    }

    void
    transformVector (Eigen::Vector3f& vin, Eigen::Vector3f& vout, Eigen::Matrix4f& transform)
    {
      Eigen::Vector4f pt;
      pt <<
              vin[0],
              vin[1],
              vin[2],
              1.0f;

      pt = transform*pt;

      vout[0] = pt[0];
      vout[1] = pt[1];
      vout[2] = pt[2];
    }
  }
}
