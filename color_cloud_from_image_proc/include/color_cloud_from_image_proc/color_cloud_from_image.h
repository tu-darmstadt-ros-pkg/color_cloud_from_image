#ifndef COLOR_CLOUD_FROM_IMAGE_H
#define COLOR_CLOUD_FROM_IMAGE_H

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <functional>

//#include <aslam/cameras/CameraGeometry.hpp>
//#include <aslam/cameras/RadialTangentialDistortion.hpp>
//#include <aslam/cameras/OmniProjection.hpp>

#include <aslam/cameras.hpp>


namespace color_cloud_from_image {

  struct IntrinsicCalibration {
    double xi, fu, fv, cu, cv, ru, rv;
    double k1, k2, p1, p2;
  };

  struct CameraModel{
    std::string name;
    IntrinsicCalibration calibration;
    sensor_msgs::ImageConstPtr last_image;
    boost::shared_ptr<aslam::cameras::CameraGeometryBase> camera;
  };


  class ColorCloudFromImage {
  public:
    ColorCloudFromImage();
    // add camera with name and calibration
    // create aslam cam instance for projection
    void addCamera(std::string name, std::string topic, const IntrinsicCalibration& calibration);
    void addCamera(std::string name, std::string topic, const ros::NodeHandle& nh);
    IntrinsicCalibration loadCalibration(const ros::NodeHandle& nh);
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

    void worldToColor(const Eigen::Vector3d& point3d);

    ros::NodeHandle nh_;
    sensor_msgs::PointCloud2 last_cloud_;
    std::map<std::string, CameraModel> cameras_;

    ros::Subscriber cloud_sub_;
    std::map<std::string, image_transport::Subscriber> cam_subs_;
    ros::Publisher cloud_pub_;
  };
}

#endif
