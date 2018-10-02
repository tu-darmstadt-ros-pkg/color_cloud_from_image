#include <color_cloud_from_image_proc/color_cloud_from_image.h>

#include <cv_bridge/cv_bridge.h>

namespace color_cloud_from_image {

ColorCloudFromImage::ColorCloudFromImage(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh){
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR); // Disable warnings, so PC copying doesn't complain about missing RGB field

  tf_buffer_.reset(new tf2_ros::Buffer());
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  self_filter_.reset(new filters::SelfFilter<pcl::PointCloud<pcl::PointXYZ> >(pnh_));

  //cloud_sub_ = nh_.subscribe("cloud", 10, &ColorCloudFromImage::cloudCallback, this);

  sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "cloud", 10);
  mn_ = new tf2_ros::MessageFilter<sensor_msgs::PointCloud2> (*sub_, *tf_buffer_, "", 30, nh_);

  self_filter_->getSelfMask()->getLinkNames(filter_frames_);
  if (filter_frames_.empty())
  {
    ROS_INFO ("No valid frames have been passed into the cloud color self filter. Will not filter for robot parts.");
    no_filter_sub_ = nh_.subscribe<sensor_msgs::PointCloud2> ("cloud", 10, boost::bind(&ColorCloudFromImage::cloudCallback, this, _1));
  }
  else
  {
    ROS_INFO ("Valid frames were passed in. We'll filter them.");
    mn_->setTargetFrames (filter_frames_);
    mn_->registerCallback (boost::bind (&ColorCloudFromImage::cloudCallback, this, _1));
  }


  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("colored_cloud", 100, false);
  cloud_debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("debug_cloud", 100, false);
}

bool ColorCloudFromImage::loadCamerasFromNamespace(ros::NodeHandle& nh) {
  if (camera_model_loader_.loadCamerasFromNamespace(nh)) {
    camera_model_loader_.startSubscribers();
    return true;
  } else {
    return false;
  }
}

void ColorCloudFromImage::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr) {
  pcl::PointCloud<pcl::PointXYZ> cloud_in;
  pcl::fromROSMsg(*cloud_ptr, cloud_in);

  pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
  std::vector<int> in_to_out_index(cloud_in.size(), -1);
  std::vector<double> distance_from_center(cloud_in.size(), camera_model::INVALID);
  // Iterate over every camera
  for (std::map<std::string, camera_model::Camera>::iterator c = camera_model_loader_.getCameraMap().begin(); c != camera_model_loader_.getCameraMap().end(); ++c) {
    camera_model::Camera& cam = c->second;
    if (cam.getLastImage()) {
      // Get transform from cloud to camera frame
      geometry_msgs::TransformStamped transform;
      std::string cam_frame_id;
      if (cam.getFrameId() != "") {
        cam_frame_id = cam.getFrameId();
      } else {
        cam_frame_id = cam.getLastImage()->header.frame_id;
      }
      try {
        transform = tf_buffer_->lookupTransform(cam_frame_id, cloud_ptr->header.frame_id, cloud_ptr->header.stamp, ros::Duration(1));
      } catch (tf2::TransformException e) {
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
      bool use_self_filter = !filter_frames_.empty();
      std::vector<int> self_filter_mask;
      if (use_self_filter){
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        self_filter_->updateWithSensorFrameAndMask(cloud, cloud_filtered, cam_frame_id,  self_filter_mask);
      }

      // Iterate over each point in cloud
      for (unsigned int i = 0; i < cloud.size(); i++) {
        if (use_self_filter && (self_filter_mask[i] != robot_self_filter::OUTSIDE) )
          continue;
        Eigen::Vector3f point_cam(cloud[i].x, cloud[i].y, cloud[i].z);
        double new_dist;
        camera_model::Color color = cam.worldToColor(point_cam.cast<double>(), new_dist);
        if (new_dist < distance_from_center[i]) {
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
          distance_from_center[i] = new_dist;
        }
      }
    }
  }

  // Convert back to sensor msg
  sensor_msgs::PointCloud2 cloud_out_msg;
  pcl::toROSMsg(cloud_out, cloud_out_msg);
  cloud_out_msg.header = cloud_ptr->header;
  cloud_pub_.publish(cloud_out_msg);
}

}
