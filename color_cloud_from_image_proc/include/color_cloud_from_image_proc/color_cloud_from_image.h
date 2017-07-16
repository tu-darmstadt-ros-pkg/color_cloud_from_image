#ifndef COLOR_CLOUD_FROM_IMAGE_H
#define COLOR_CLOUD_FROM_IMAGE_H

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <functional>

#include <aslam/cameras.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "robot_self_filter/self_see_filter.h"

#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>


namespace color_cloud_from_image {

  using namespace aslam::cameras;

  constexpr double INVALID = std::numeric_limits<double>::max();

  struct Color {
    Color() : r(0), g(0), b(0) {}
    Color(uint8_t _r, uint8_t _g, uint8_t _b)
      : r(_r), g(_g), b(_b) {}

    double r, g, b;
  };

  struct IntrinsicCalibration {
//    double xi, fu, fv, cu, cv, ru, rv;
//    double k1, k2, p1, p2;
    std::string camera_model;
    std::vector<double> intrinsics;
    std::string distortion_model;
    std::vector<double> distortion_coeffs;
    std::vector<int> resolution;
  };

  struct Camera {
    std::string name;
    IntrinsicCalibration calibration;
    sensor_msgs::ImageConstPtr last_image;
    boost::shared_ptr<aslam::cameras::CameraGeometryBase> camera_model;
    image_transport::Subscriber sub;
    std::string frame_id; //overrides header.frame_id if set
  };


  class ColorCloudFromImage {
  public:
    ColorCloudFromImage();
    // add camera with name and calibration
    // create aslam cam instance for projection
    void loadCamerasFromNamespace(ros::NodeHandle &nh);
//    void addCamera(std::string name, std::string topic, std::string frame_id, const IntrinsicCalibration& calibration);
    void loadCamera(std::string name , ros::NodeHandle &nh);
    bool loadCalibration(ros::NodeHandle &nh, IntrinsicCalibration &calibration);
  private:
    // save images for respective cam
    void imageCallback(std::string cam_name, const sensor_msgs::ImageConstPtr& image_ptr);
    /* on new pc:
     * 1. iterate over each point
     * 2. transform point to cam frame
     * 3. project point to each cam until first success (aslam cam)
     * 4. save pixel color for point
     * 5. republish cloud
     */
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr);

    double worldToColor(const Eigen::Vector3d& point3d, const Camera &cam, Color &color); // returns distance to middle

    //boost::shared_ptr<CameraGeometryBase> createCameraGeometry(const Camera &cam);

    std::string intrinsicsToString(const IntrinsicCalibration& calibration);
    template<typename T> bool getParam(ros::NodeHandle& nh, const std::string& key, T& var) const {
      if (!nh.getParam(key, var)) {
        ROS_ERROR_STREAM("Could not get parameter '" + nh.getNamespace() + "/" << key << "'");
        return false;
      } else {
        return true;
      }
    }

    template<typename T>
    std::string vecToString(const std::vector<T>& vec) {
      std::stringstream ss;
      ss << "[";
      for (unsigned int i = 0; i < vec.size(); ++i) {
        ss << vec[i];
        if (i != vec.size() -1) {
          ss << ", ";
        }
      }
      ss << "]";
      return ss.str();
    }

    ros::NodeHandle nh_;
    sensor_msgs::PointCloud2 last_cloud_;
    std::map<std::string, Camera> cameras_;

    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    boost::shared_ptr<filters::SelfFilter<pcl::PointCloud<pcl::PointXYZ> > > self_filter_;

    //ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    ros::Publisher cloud_debug_pub_;

    std::vector<std::string> filter_frames_;

    tf2_ros::MessageFilter<sensor_msgs::PointCloud2>           *mn_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_;

    ros::Subscriber no_filter_sub_;

  };
}

#endif
