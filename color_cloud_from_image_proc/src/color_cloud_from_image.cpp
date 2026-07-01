#include <color_cloud_from_image_proc/color_cloud_from_image.h>

#include <cv_bridge/cv_bridge.hpp>

namespace color_cloud_from_image {

ColorCloudFromImage::ColorCloudFromImage(const rclcpp::NodeOptions& options)
  : node_(std::make_shared<rclcpp::Node>("color_cloud_from_image_proc", options)), lazy_(true), enabled_(false), camera_loader_(node_) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR); // Disable warnings, so PC copying doesn't complain about missing RGB field

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(),
    node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

  //self_filter_ = std::make_shared<filters::SelfFilter<pcl::PointCloud<pcl::PointXYZ>>(node);

  sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(node_, "cloud");//, 10);
  //sub_->registerCallback(std::bind(&ColorCloudFromImage::cloudCallback, this, std::placeholders::_1));
  mn_ = new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> (*sub_, *tf_buffer_, "", 30, node_);

  // Load parameters
  node_->declare_parameter("lazy", true);
  node_->get_parameter("lazy", lazy_);
  node_->declare_parameter("max_time_diff", 0.1);
  node_->get_parameter("max_time_diff", max_time_diff_);
  enabled_ = !lazy_;
  node_->declare_parameter("mono", false);
  node_->get_parameter("mono", mono_);

  std::function<void(sensor_msgs::msg::PointCloud2::ConstSharedPtr)> cloud_callback;
  if (mono_) {
    cloud_callback = std::bind(&ColorCloudFromImage::cloudCallback<pcl::PointXYZI>, this, std::placeholders::_1);
  } else {
    cloud_callback = std::bind(&ColorCloudFromImage::cloudCallback<pcl::PointXYZRGB>, this, std::placeholders::_1);
  }

  // TODO add self filter
  //self_filter_->getSelfMask()->getLinkNames(filter_frames_);
  use_self_filter_ = !filter_frames_.empty();
  if (use_self_filter_)
  {
    RCLCPP_INFO (node_->get_logger(), "Valid frames were passed in. We'll filter them.");
    mn_->setTargetFrames (filter_frames_);

    mn_->registerCallback (cloud_callback);
  }
  else
  {
    RCLCPP_INFO (node_->get_logger(), "No valid frames have been passed into the cloud color self filter. Will not filter for robot parts.");
    no_filter_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2> ("cloud", 10, cloud_callback);
  }

  

  if (enabled_) {
    startSubscribers();
  } else {
    stopSubscribers();
  }
  rclcpp::PublisherEventCallbacks event_callbacks;
  event_callbacks.matched_callback = std::bind(&ColorCloudFromImage::connectCb, this, std::placeholders::_1);
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks = event_callbacks;
  cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cloud", 100, pub_options);
  cloud_debug_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("debug_cloud", 100);
}

template<typename PointType>
void ColorCloudFromImage::cloudCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2 const> cloud_ptr) {
  if (!enabled_) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud_in;
  pcl::fromROSMsg(*cloud_ptr, cloud_in);

  pcl::PointCloud<PointType> cloud_out;
  std::vector<int> in_to_out_index(cloud_in.size(), -1);
  std::vector<double> distance_from_center(cloud_in.size(), extended_image_geometry::INVALID);
  // Iterate over every camera
  for (const extended_image_geometry::CameraPtr& cam: camera_loader_.cameras()) {
    if (!cam->cameraInfoReceived()) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, "Camera info not received for camera: " << cam->getName());
      continue;
    }
    if (!cam->getLastImage()) {
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, "No image received for camera: " << cam->getName());
      continue;
    }
    rclcpp::Time cloud_time = cloud_ptr->header.stamp;
    rclcpp::Time image_time = cam->getLastStamp();
    if (std::abs((cloud_time - image_time).seconds()) > max_time_diff_) {
      RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Time difference too large for camera: " << cam->getName() << ". Cloud time to image time difference: " << (cloud_time - image_time).seconds());
      continue;
    }
    
    cv_bridge::CvImageConstPtr cv_image;
    if (mono_) {
      cv_image = cam->getLastImageCvMono();
    } else {
      cv_image = cam->getLastImageCv();
    }
    if (!cv_image) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, "Image conversion failed for camera: " << cam->getName());
      continue;
    }
    // Get transform from cloud to camera frame
    geometry_msgs::msg::TransformStamped transform;
    std::string cam_frame_id;
    if (!cam->model().cameraInfo()->frame_id.empty()) {
      cam_frame_id = cam->model().cameraInfo()->frame_id;
    } else {
      cam_frame_id = cv_image->header.frame_id;
    }
    try {
      transform = tf_buffer_->lookupTransform(cam_frame_id, cloud_ptr->header.frame_id, cloud_ptr->header.stamp, rclcpp::Duration(1, 0));
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "LookupTransform failed. Reason: " << e.what());
      continue;
    }

    // Transform cloud to camera frame
    sensor_msgs::msg::PointCloud2 cloud_cam_frame;
    tf2::doTransform(*cloud_ptr, cloud_cam_frame, transform);
    cloud_cam_frame.header.frame_id = cam_frame_id;

    // Convert to pcl
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(cloud_cam_frame, cloud);

    // Call self filter
    std::vector<int> self_filter_mask;

    // TODO add new self filter
    /*if (use_self_filter_) {
      pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
      self_filter_->updateWithSensorFrameAndMask(cloud, cloud_filtered, cam_frame_id,  self_filter_mask);
    }*/

    // Iterate over each point in cloud
    for (unsigned int i = 0; i < cloud.size(); i++) {
      // TODO port self filter
      /*if (use_self_filter_ && (self_filter_mask[i] != robot_self_filter::OUTSIDE))
        continue;*/
      Eigen::Vector3f point_cam(cloud[i].x, cloud[i].y, cloud[i].z);
      double new_dist;
      
      if (new_dist < distance_from_center[i]) {
        // Distance to image center is lower, set/update color of point
        // Find point in cloud out
        int cloud_out_idx = in_to_out_index[i];
        if (cloud_out_idx == -1) {
          // Point not in cloud out yet
          PointType colored_point;
          colored_point.x = cloud_in[i].x;
          colored_point.y = cloud_in[i].y;
          colored_point.z = cloud_in[i].z;
          cloud_out.push_back(colored_point);
          in_to_out_index[i] = static_cast<int>(cloud_out.size()-1);
          cloud_out_idx = in_to_out_index[i];
        }
        // Update color
        PointType& colored_point = cloud_out[static_cast<size_t>(cloud_out_idx)];
        color_point(colored_point, cam, point_cam, cv_image, new_dist);
        distance_from_center[i] = new_dist;
      }
    }
  }

  // Convert back to sensor msg
  sensor_msgs::msg::PointCloud2::UniquePtr cloud_out_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(cloud_out, *cloud_out_msg);
  cloud_out_msg->header = cloud_ptr->header;
  cloud_pub_->publish(std::move(cloud_out_msg));
}

template<>
void ColorCloudFromImage::color_point<pcl::PointXYZRGB>(pcl::PointXYZRGB& point, const extended_image_geometry::CameraPtr& cam, const Eigen::Vector3f& point_cam, const cv_bridge::CvImageConstPtr& cv_image, double& new_dist) {
  extended_image_geometry::Color color = cam->model().worldToColor<extended_image_geometry::Color>(point_cam.cast<double>(), cv_image->image, new_dist);
  point.r = color.r;
  point.g = color.g;
  point.b = color.b;
}

template<>
void ColorCloudFromImage::color_point<pcl::PointXYZI>(pcl::PointXYZI& point, const extended_image_geometry::CameraPtr& cam, const Eigen::Vector3f& point_cam, const cv_bridge::CvImageConstPtr& cv_image, double& new_dist) {
  float mono = cam->model().worldToColor<float>(point_cam.cast<double>(), cv_image->image, new_dist);
  point.intensity = mono;
}


void ColorCloudFromImage::connectCb(rclcpp::MatchedInfo& info)
{
  if (!lazy_) {
    return;
  }
  if (info.current_count == 0 && enabled_) {
    enabled_ = false;
    stopSubscribers();
  } else {
    if (!enabled_) {
      enabled_ = true;
      startSubscribers();
    }
  }
}

void ColorCloudFromImage::startSubscribers()
{
  camera_loader_.startImageSubscribers();
  //sub_->subscribe(node_, "cloud", rclcpp::QoS(10));
  sub_->subscribe();

  std::function<void(sensor_msgs::msg::PointCloud2::ConstSharedPtr)> cloud_callback;
  if (mono_) {
    cloud_callback = std::bind(&ColorCloudFromImage::cloudCallback<pcl::PointXYZI>, this, std::placeholders::_1);
  } else {
    cloud_callback = std::bind(&ColorCloudFromImage::cloudCallback<pcl::PointXYZRGB>, this, std::placeholders::_1);
  }

  if (!use_self_filter_) {
    no_filter_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2> ("cloud", 10, cloud_callback);
  }
}

void ColorCloudFromImage::stopSubscribers()
{
  camera_loader_.stopImageSubscribers();
  sub_->unsubscribe();
  if (!use_self_filter_) {
    no_filter_sub_.reset();
  }
}

}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(color_cloud_from_image::ColorCloudFromImage)