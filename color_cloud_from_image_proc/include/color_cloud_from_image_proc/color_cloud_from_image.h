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

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>


namespace color_cloud_from_image {

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
    std::vector<double> distortion;
    std::pair<int, int> resolution;
  };

  struct Camera {
    std::string name;
    IntrinsicCalibration calibration;
    sensor_msgs::ImageConstPtr last_image;
    boost::shared_ptr<aslam::cameras::CameraGeometryBase> camera_model;
    image_transport::Subscriber sub;
  };


  class ColorCloudFromImage {
  public:
    ColorCloudFromImage();
    // add camera with name and calibration
    // create aslam cam instance for projection
    void loadCamerasFromNamespace(ros::NodeHandle &nh);
    void addCamera(std::string name, std::string topic, const IntrinsicCalibration& calibration);
    IntrinsicCalibration loadCalibration(ros::NodeHandle &nh);
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

    bool worldToColor(const Eigen::Vector3d& point3d, const Camera &cam, Color &color);

    ros::NodeHandle nh_;
    sensor_msgs::PointCloud2 last_cloud_;
    std::map<std::string, Camera> cameras_;

    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
  };
}

#endif
