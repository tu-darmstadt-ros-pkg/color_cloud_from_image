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

#include <camera_model_loader/camera_model_loader.h>


namespace color_cloud_from_image {

  using namespace aslam::cameras;

  class ColorCloudFromImage {
  public:
    ColorCloudFromImage();
    bool loadCamerasFromNamespace(ros::NodeHandle &nh);
  private:
    /* on new pc:
     * 1. iterate over each point
     * 2. transform point to cam frame
     * 3. project point to each cam until first success (aslam cam)
     * 4. save pixel color for point
     * 5. republish cloud
     */
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr);

    ros::NodeHandle nh_;
    sensor_msgs::PointCloud2 last_cloud_;

    camera_model::CameraModelLoader camera_model_loader_;
    boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    ros::Publisher cloud_debug_pub_;

  };
}

#endif
