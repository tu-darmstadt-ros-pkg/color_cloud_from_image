#ifndef COLOR_CLOUD_FROM_IMAGE_H
#define COLOR_CLOUD_FROM_IMAGE_H

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

namespace color_cloud_from_image {

  struct IntrinsicCalibration {};

  struct CameraModel{
    std::string name;
    std::string frame_id;
    IntrinsicCalibration calibration;
    sensor_msgs::Image last_image_;
  };


  class ColorCloudFromImage {
  public:
    // add camera with name and calibration
    // create aslam cam instance for projection
    void addCamera(std::string name, const IntrinsicCalibration& calibration);
    IntrinsicCalibration loadCalibration(const ros::NodeHandle& nh);
  private:
    // save images for respective cam
    void imageCB(std::string cam_name, const sensor_msgs::ImageConstPtr& image_ptr);
    /* on new pc:
     * 1. iterate over each point
     * 2. transform point to cam frame
     * 3. project point to each cam until first success (aslam cam)
     * 4. save pixel color for point
     * 5. republish cloud
     */
    void pointcloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr);

    void worldToColor(const Eigen::Vector3d& point3d);
    Eigen::Vector2d worldToCam(const Eigen::Vector3d& point3d);


    sensor_msgs::PointCloud2 last_cloud_;
    std::vector<CameraModel> cameras_;
  };
}

#endif
