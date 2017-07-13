#include <color_cloud_from_image_proc/color_cloud_from_image.h>

#include <cv_bridge/cv_bridge.h>

namespace color_cloud_from_image {

ColorCloudFromImage::ColorCloudFromImage() {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR); // Disable warnings, so PC copying doesn't complain about missing RGB field
  nh_ = ros::NodeHandle();

  tf_buffer_.reset(new tf2_ros::Buffer());
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));


  cloud_sub_ = nh_.subscribe("cloud", 10, &ColorCloudFromImage::cloudCallback, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("colored_cloud", 1000, false);
  cloud_debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("debug_cloud", 1000, false);
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

  std::vector<double> sqr_dist(cloud_ptr->height * cloud_ptr->width, INVALID);
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

      // iterate over each point in cloud
      for (unsigned int i = 0; i < cloud.size(); i++) {
        const pcl::PointXYZ& point = cloud[i];

        Eigen::Vector3d point_cam(point.x, point.y, point.z);
        Color color;
        double new_dist = worldToColor(point_cam, cam, color);
        if (new_dist < sqr_dist[i]) {
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

double ColorCloudFromImage::worldToColor(const Eigen::Vector3d& point3d, const camera_model::Camera& cam, Color& color) {
  boost::shared_ptr<aslam::cameras::CameraGeometryBase> cam_model = cam.camera_model;
  Eigen::VectorXd pixel(2);
  if (cam_model->vsEuclideanToKeypoint(point3d, pixel)) {
    cv_bridge::CvImageConstPtr cv_image;
    try
    {
      cv_image = cv_bridge::toCvShare(cam.last_image);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR_STREAM("Conversion failed: " << e.what());
      return false;
    }
    const cv::Mat& img = cv_image->image;
    Eigen::Vector2i pixel_i(std::round(pixel(0)), std::round(pixel(1)));
    cv::Vec3b color_vec = img.at<cv::Vec3b>(pixel_i(1), pixel_i(0));
    color.r = color_vec[0];
    color.g = color_vec[1];
    color.b = color_vec[2];

    double sqr_dist = std::pow(pixel(0) - img.rows / 2.0, 2) + std::pow(pixel(1) - img.cols / 2.0, 2);

    return sqr_dist;
  } else {
    return INVALID; // kinda hacky?
  }
}

}
