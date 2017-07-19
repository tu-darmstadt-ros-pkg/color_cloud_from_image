#include <color_cloud_from_image_proc/color_cloud_from_image.h>

#include <cv_bridge/cv_bridge.h>

namespace color_cloud_from_image {

ColorCloudFromImage::ColorCloudFromImage() {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR); // Disable warnings, so PC copying doesn't complain about missing RGB field
  nh_ = ros::NodeHandle();

  tf_buffer_.reset(new tf2_ros::Buffer());
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  pnh_ = ros::NodeHandle("~");

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
  return camera_model_loader_.loadCamerasFromNamespace(nh);
}

void ColorCloudFromImage::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
  pcl::fromROSMsg(*cloud_ptr, cloud_out); // complains about missing RGB field if pcl warnings are not disabled

  // set default color for all pixels without color
  for (unsigned int i = 0; i < cloud_out.size(); i++) {
    cloud_out[i].r = 255;
    cloud_out[i].g = 0;
    cloud_out[i].b = 0;
  }

  std::vector<double> confidence(cloud_ptr->height * cloud_ptr->width, -1);
  for (std::map<std::string, camera_model::Camera>::const_iterator c = camera_model_loader_.getCameraMap().begin(); c != camera_model_loader_.getCameraMap().end(); ++c) {
    const camera_model::Camera& cam = c->second;
    if (cam.last_image) {
      // get transform to camera frame
      geometry_msgs::TransformStamped transform;
      std::string cam_frame_id;
      if (cam.frame_id != "") {
        cam_frame_id = cam.frame_id;
      } else {
        cam_frame_id = cam.last_image->header.frame_id;
      }

      try {
        transform = tf_buffer_->lookupTransform(cam_frame_id, cloud_ptr->header.frame_id, cloud_ptr->header.stamp, ros::Duration(1));
      } catch (tf2::TransformException e) {
        ROS_WARN_STREAM("LookupTransform failed. Reason: " << e.what());
        continue;
      }
      // transform cloud to cam frame
      sensor_msgs::PointCloud2 cloud_cam_frame;
      tf2::doTransform(*cloud_ptr, cloud_cam_frame, transform); // TODO transform points individually after checking if they already have a color
      cloud_cam_frame.header.frame_id = cam_frame_id;
      if (cam.name == "cam0") {
        cloud_debug_pub_.publish(cloud_cam_frame);
      }
      // transform to pcl
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(cloud_cam_frame, cloud);

      bool use_self_filter = !filter_frames_.empty();

      std::vector<int> self_filter_mask;

      if (use_self_filter){
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        self_filter_->updateWithSensorFrameAndMask(cloud, cloud_filtered, cam_frame_id,  self_filter_mask);
      }

      // iterate over each point in cloud
      for (unsigned int i = 0; i < cloud.size(); i++) {

        //if (use_self_filter && (self_filter_mask[i] != robot_self_filter::OUTSIDE) )
        //  continue;

        const pcl::PointXYZ& point = cloud[i];

        Eigen::Vector3d point_cam(point.x, point.y, point.z);
        double new_confidence;
        camera_model::Color color = cam.worldToColor(point_cam, new_confidence);
        if (new_confidence > confidence[i]) {
          //ROS_INFO_STREAM("Found color! (" << color.r << ", " << color.g << ", " << color.b << ")");
          cloud_out[i].r = color.r;
          cloud_out[i].g = color.g;
          cloud_out[i].b = color.b;
        }
      }
    }
  }

  sensor_msgs::PointCloud2 cloud_out_msg;
  pcl::toROSMsg(cloud_out, cloud_out_msg);
  cloud_pub_.publish(cloud_out_msg);
}

}
