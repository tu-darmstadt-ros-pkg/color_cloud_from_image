#ifndef COLOR_CLOUD_FROM_IMAGE_H
#define COLOR_CLOUD_FROM_IMAGE_H

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Eigen>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
//#include <image_transport/image_transport.h>

#include <functional>

//#include <aslam/cameras.hpp>
#include <extended_camera_model/camera_model.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// TODO switch to gpu filter or use https://github.com/leggedrobotics/robot_self_filter
//#include "robot_self_filter/self_see_filter.h"

#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

#include <extended_camera_loader/camera_loader.h>

#include "tf2_ros/create_timer_ros.h"


namespace color_cloud_from_image {

  class ColorCloudFromImage {
  public:
    ColorCloudFromImage(const rclcpp::NodeOptions& options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const
    {
      return this->node_->get_node_base_interface();
    }
  private:
    /* on new pc:
     * 1. iterate over each point
     * 2. transform point to cam frame
     * 3. project point to each cam until first success (aslam cam)
     * 4. save pixel color for point
     * 5. republish cloud
     */
    template<typename PointType>
    void cloudCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2 const> cloud_ptr);

    template<typename PointType, typename ColorType>
    void processCamera(const extended_image_geometry::CameraPtr& cam, const cv_bridge::CvImageConstPtr& cv_image, const pcl::PointCloud<pcl::PointXYZ>& cloud_in, const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<PointType>& cloud_out, std::vector<int>& in_to_out_index, std::vector<double>& distance_from_center);
    /*template<typename PointType, typename ImageType>
    void color_point(PointType& point, const extended_image_geometry::CameraPtr& cam, const Eigen::Vector3f& point_cam, const cv_bridge::CvImageConstPtr& cv_image, double& new_dist);*/

    template<typename PointType, typename ColorType>
    void update_point_color(PointType& point_to_update, const ColorType& color);

    void connectCb(rclcpp::MatchedInfo& info);

    void startSubscribers();
    void stopSubscribers();

    rclcpp::Node::SharedPtr node_;

    bool lazy_;
    bool enabled_;

    bool mono_;

    sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;

    extended_image_geometry::CameraLoader camera_loader_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    //std::shared_ptr<filters::SelfFilter<pcl::PointCloud<pcl::PointXYZ> > > self_filter_;
    bool use_self_filter_;

    //ros::Subscriber cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_debug_pub_;

    std::vector<std::string> filter_frames_;

    tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> *mn_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr no_filter_sub_;

    double max_time_diff_;

  };
}

#endif
