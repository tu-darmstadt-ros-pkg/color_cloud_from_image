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

#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <tf2_ros/message_filter.hpp>
#include <message_filters/subscriber.hpp>

#include <deque>
#include <mutex>

#include <extended_camera_loader/camera_loader.h>

#include "tf2_ros/create_timer_ros.hpp"


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
    void processCamera(const extended_image_geometry::CameraPtr& cam, const cv_bridge::CvImageConstPtr& cv_image, const pcl::PointCloud<pcl::PointXYZ>& cloud_in, const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<PointType>& cloud_out, std::vector<int>& in_to_out_index, std::vector<double>& distance_from_center, const cv::Mat& self_mask);

    /**
     * Latest self mask for a camera.
     * The mask is rendered for the timestamp of a particular image.
     *
     * @return false when the mask is missing or belongs to a different frame, in which case the
     *         camera must not be used at all: colouring from an unverified image is what leaks the
     *         robot into the map.
     */
    bool getSelfMask(size_t camera_index, const std::string& camera_name, const rclcpp::Time& image_stamp, cv::Mat& mask_out) const;
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

    bool use_self_filter_;
    
    bool use_self_mask_;
    bool keep_uncolored_;
    mutable std::mutex self_mask_mutex_;
    std::vector<std::deque<sensor_msgs::msg::Image::ConstSharedPtr>> self_masks_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> self_mask_subs_;
    static constexpr size_t kSelfMaskHistory = 5;

    //ros::Subscriber cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_debug_pub_;

    std::vector<std::string> filter_frames_;

    tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> *mn_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr no_filter_sub_;

    double max_time_diff_;

    bool mono_scaling_;
    double mono_scale_;
    double mono_offset_;

  };
}

#endif
