#include <color_cloud_from_image_proc/color_cloud_from_image.h>

#include <cv_bridge/cv_bridge.h>

namespace color_cloud_from_image {

ColorCloudFromImage::ColorCloudFromImage(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), lazy_(true), enabled_(false), camera_loader_(nh, pnh) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR); // Disable warnings, so PC copying doesn't complain about missing RGB field

  ROS_INFO("color_cloud init");

  tf_buffer_.reset(new tf2_ros::Buffer());
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  self_filter_.reset(new filters::SelfFilter<pcl::PointCloud<pcl::PointXYZ> >(pnh_));

  sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>();
  mn_ = new tf2_ros::MessageFilter<sensor_msgs::PointCloud2> (*sub_, *tf_buffer_, "", 30, nh_);

  self_filter_->getSelfMask()->getLinkNames(filter_frames_);
  use_self_filter_ = !filter_frames_.empty();
  use_self_filter_ = false;
  if (use_self_filter_)
  {
    ROS_INFO ("Valid frames were passed in. We'll filter them.");
    mn_->setTargetFrames (filter_frames_);
    mn_->registerCallback (boost::bind (&ColorCloudFromImage::cloudCallback, this, _1));
  }
  else
  {
    ROS_INFO ("No valid frames have been passed into the cloud color self filter. Will not filter for robot parts.");
//    no_filter_sub_ = nh_.subscribe<sensor_msgs::PointCloud2> ("cloud", 10, boost::bind(&ColorCloudFromImage::cloudCallback, this, _1));
  }

  // Load parameters
  pnh_.param("lazy", lazy_, true);
  enabled_ = !lazy_;

  if (enabled_) {
    startSubscribers();
  }

  ros::SubscriberStatusCallback connect_cb = boost::bind(&ColorCloudFromImage::connectCb, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("colored_cloud", 100, connect_cb, connect_cb);
  cloud_debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("debug_cloud", 100);
}

void ColorCloudFromImage::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr) {
  if (!enabled_) {
    return;
  }
  if (!camera_loader_.cameraInfosReceived()) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> cloud_in;
  pcl::fromROSMsg(*cloud_ptr, cloud_in);

  pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
  cloud_out.width = cloud_in.width;
  cloud_out.height = cloud_in.height;
  std::vector<int> in_to_out_index(cloud_in.size(), -1);
  std::vector<double> distance_from_center(cloud_in.size(), kalibr_image_geometry::INVALID);
  // Iterate over every camera
  for (const kalibr_image_geometry::CameraPtr& cam: camera_loader_.cameras()) {
    if (cam->getLastImage()) {
      cv_bridge::CvImageConstPtr cv_image = cam->getLastImageCv();
      // Get transform from cloud to camera frame
      geometry_msgs::TransformStamped transform;
      std::string cam_frame_id;
      if (!cam->model().cameraInfo().frame_id.empty()) {
        cam_frame_id = cam->model().cameraInfo().frame_id;
      } else {
        cam_frame_id = cv_image->header.frame_id;
      }
      try {
        transform = tf_buffer_->lookupTransform(cam_frame_id, cloud_ptr->header.frame_id, cloud_ptr->header.stamp, ros::Duration(1));
      } catch (const tf2::TransformException& e) {
        ROS_WARN_STREAM("LookupTransform failed. Reason: " << e.what());
        continue;
      }

      // Transform cloud to camera frame
      sensor_msgs::PointCloud2 cloud_cam_frame;
      tf2::doTransform(*cloud_ptr, cloud_cam_frame, transform);
      cloud_cam_frame.header.frame_id = cam_frame_id;

      // Convert to pcl
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(cloud_cam_frame, cloud);

      // Call self filter
      std::vector<int> self_filter_mask;
      if (use_self_filter_) {
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        self_filter_->updateWithSensorFrameAndMask(cloud, cloud_filtered, cam_frame_id,  self_filter_mask);
      }

      // Iterate over each point in cloud
      for (unsigned int i = 0; i < cloud.size(); i++) {
        if (use_self_filter_ && (self_filter_mask[i] != robot_self_filter::OUTSIDE))
          continue;
        Eigen::Vector3f point_cam(cloud[i].x, cloud[i].y, cloud[i].z);
        double new_dist;
        kalibr_image_geometry::Color color = cam->model().worldToColor(point_cam.cast<double>(), cv_image->image, new_dist);
        /*if (new_dist < distance_from_center[i])*/ {
          // Distance to image center is lower, set/update color of point
          // Find point in cloud out
          int cloud_out_idx = in_to_out_index[i];
          if (cloud_out_idx == -1) {
            // Point not in cloud out yet
            pcl::PointXYZRGB colored_point;
            colored_point.x = cloud_in[i].x;
            colored_point.y = cloud_in[i].y;
            colored_point.z = cloud_in[i].z;
            cloud_out.push_back(colored_point);
            in_to_out_index[i] = static_cast<int>(cloud_out.size()-1);
            cloud_out_idx = in_to_out_index[i];
          }
          // Update color
          pcl::PointXYZRGB& colored_point = cloud_out[static_cast<size_t>(cloud_out_idx)];
          colored_point.r = color.r;
          colored_point.g = color.g;
          colored_point.b = color.b;
          if (new_dist < distance_from_center[i])
            distance_from_center[i] = new_dist;
        }
      }
    }
  }

  // Convert back to sensor msg
  sensor_msgs::PointCloud2Ptr cloud_out_msg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(cloud_out, *cloud_out_msg);
  cloud_out_msg->header = cloud_ptr->header;
  cloud_out_msg->width = cloud_ptr->width;
  cloud_out_msg->height = cloud_ptr->height;
  cloud_pub_.publish(cloud_out_msg);
}

void ColorCloudFromImage::connectCb()
{
  if (!lazy_) {
    return;
  }
  if (cloud_pub_.getNumSubscribers() == 0) {
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
  sub_->subscribe(nh_, "cloud", 10);
  if (!use_self_filter_) {
    no_filter_sub_ = nh_.subscribe<sensor_msgs::PointCloud2> ("cloud", 10, boost::bind(&ColorCloudFromImage::cloudCallback, this, _1));
  }
}

void ColorCloudFromImage::stopSubscribers()
{
  camera_loader_.stopImageSubscribers();
  sub_->unsubscribe();
  if (!use_self_filter_) {
    no_filter_sub_.shutdown();
  }
}

}
