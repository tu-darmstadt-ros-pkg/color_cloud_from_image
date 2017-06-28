#include <color_cloud_from_image_proc/color_cloud_from_image.h>

#include <cv_bridge/cv_bridge.h>

namespace color_cloud_from_image {

ColorCloudFromImage::ColorCloudFromImage() {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR); // Disable warnings, so PC copying doesn't complain about missing RGB field
  nh_ = ros::NodeHandle();

  it_.reset(new image_transport::ImageTransport(nh_));
  tf_buffer_.reset(new tf2_ros::Buffer());
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));


  cloud_sub_ = nh_.subscribe("cloud", 10, &ColorCloudFromImage::cloudCallback, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("colored_cloud", 1000, false);
  cloud_debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("debug_cloud", 1000, false);
}

void ColorCloudFromImage::loadCamerasFromNamespace(ros::NodeHandle& nh) {
  ROS_INFO_STREAM("Loading parameters from namespace " << nh.getNamespace() + "/cameras");
  XmlRpc::XmlRpcValue cams;
  nh.getParam("cameras", cams);
  ROS_ASSERT(cams.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = cams.begin(); it != cams.end(); ++it) {
    std::string cam_name = (std::string)(it->first);
    ros::NodeHandle cam_nh(nh, "cameras/" + cam_name);
    loadCamera(cam_name, cam_nh);
  }
}

bool ColorCloudFromImage::loadCalibration(ros::NodeHandle& nh, IntrinsicCalibration& calibration) {
  ROS_INFO_STREAM("Loading intrinsics from nh: " << nh.getNamespace());
  bool valid = true;
  valid = valid && getParam<std::string>(nh, "camera_model", calibration.camera_model);
  valid = valid && getParam<std::vector<double>>(nh, "intrinsics", calibration.intrinsics);
  valid = valid && getParam<std::string>(nh, "distortion_model", calibration.distortion_model);
  valid = valid && getParam<std::vector<double>>(nh, "distortion_coeffs", calibration.distortion_coeffs);
  valid = valid && getParam<std::vector<int>>(nh, "resolution", calibration.resolution);
  return valid;
}

void ColorCloudFromImage::loadCamera(std::string name, ros::NodeHandle &nh) {
  Camera cam;
  cam.name = name;
  std::string rostopic;
  if (!nh.getParam("rostopic", rostopic)) {
    ROS_ERROR_STREAM("Could not get parameter " + nh.getNamespace() + "/rostopic");
    return;
  }

  nh.param<std::string>("frame_id", cam.frame_id, ""); //overrides image frame_id (optional)
  bool success = loadCalibration(nh, cam.calibration);
  if (!success) {
    ROS_ERROR_STREAM("Could not get intrinsic calibration in ns '" << nh.getNamespace() << "'");
  }
  RadialTangentialDistortion distortion(cam.calibration.distortion_coeffs[0], cam.calibration.distortion_coeffs[1],
                                        cam.calibration.distortion_coeffs[2], cam.calibration.distortion_coeffs[3]);
  OmniProjection<RadialTangentialDistortion> projection(cam.calibration.intrinsics[0], cam.calibration.intrinsics[1], cam.calibration.intrinsics[2],
                                                        cam.calibration.intrinsics[3], cam.calibration.intrinsics[4], cam.calibration.resolution[0],
                                                        cam.calibration.resolution[1], distortion);
  cam.camera_model.reset(new CameraGeometry<OmniProjection<RadialTangentialDistortion>, GlobalShutter, NoMask>(projection));
  std::pair<std::string, Camera> entry(name, cam);
  ROS_INFO_STREAM("Found cam: " << cam.name << std::endl
                  << " -- topic: " << rostopic << std::endl
                  << " -- frame_id: " << cam.frame_id << std::endl
                  << intrinsicsToString(cam.calibration));
  std::pair<std::map<std::string, Camera>::iterator, bool> result = cameras_.emplace(entry);
  if (!result.second) {
    ROS_WARN_STREAM("Couldn't create camera of name '" << cam.name << "' because it already existed.");
    return;
  }
  result.first->second.sub = it_->subscribe(rostopic, 1, boost::bind(&ColorCloudFromImage::imageCallback, this, name, _1));
}

boost::shared_ptr<CameraGeometryBase> ColorCloudFromImage::createCameraGeometry(const Camera& cam) {
  if (cam.calibration.distortion_model == "radtan") {
//    typedef typename distortion_t RadialTangentialDistortion;
//    distortion_t distortion(cam.calibration.distortion_coeffs[0], cam.calibration.distortion_coeffs[1],
//        cam.calibration.distortion_coeffs[2], cam.calibration.distortion_coeffs[3]);
  }

}

//void ColorCloudFromImage::addCamera(std::string name, std::string topic, std::string frame_id, const IntrinsicCalibration& calibration) {
//  Camera cam;
//  cam.name = name;
//  cam.frame_id = frame_id;
//  cam.calibration = calibration;

//  // TODO replace with factory dependent on cam and distortion model
//  // TODO check vector lengths first
//  RadialTangentialDistortion distortion(calibration.distortion_coeffs[0], calibration.distortion_coeffs[1],
//                                        calibration.distortion_coeffs[2], calibration.distortion_coeffs[3]);
//  OmniProjection<RadialTangentialDistortion> projection(calibration.intrinsics[0], calibration.intrinsics[1], calibration.intrinsics[2],
//                                                        calibration.intrinsics[3], calibration.intrinsics[4], calibration.resolution[0],
//                                                        calibration.resolution[1], distortion);
//  cam.camera_model.reset(new CameraGeometry<OmniProjection<RadialTangentialDistortion>, GlobalShutter, NoMask>(projection));

//  cam.sub = it_->subscribe(topic, 1, boost::bind(&ColorCloudFromImage::imageCallback, this, name, _1));
//  std::pair<std::string, Camera> entry(name, cam);
//  ROS_INFO_STREAM("Found cam: " << cam.name << std::endl
//                  << " -- topic: " << topic << std::endl
//                  << " -- frame_id: " << frame_id << std::endl
//                  << intrinsicsToString(calibration));
//  cameras_.emplace(entry);
//}

void ColorCloudFromImage::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
  pcl::fromROSMsg(*cloud_ptr, cloud_out); // complains about missing RGB field if pcl warnings are not disabled

  std::vector<bool> has_color(cloud_ptr->height * cloud_ptr->width, false);
  for (std::map<std::string, Camera>::iterator c = cameras_.begin(); c != cameras_.end(); ++c) {
    const Camera& cam = c->second;
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
        // skip if point already has a color
        if (has_color[i]) {
          continue;
        }

        Eigen::Vector3d point_cam(point.x, point.y, point.z);
        Color color;
        has_color[i] = worldToColor(point_cam, cam, color);
        if (has_color[i]) {
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

void ColorCloudFromImage::imageCallback(std::string cam_name, const sensor_msgs::ImageConstPtr& image_ptr) {
  try {
    cameras_.at(cam_name).last_image = image_ptr;
  } catch (std::out_of_range) {
    ROS_ERROR_STREAM("Could not find cam " << cam_name << ". This should not have happened. Please contact the maintainer.");
  }
}

bool ColorCloudFromImage::worldToColor(const Eigen::Vector3d& point3d, const Camera& cam, Color& color) {
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
//    Eigen::Vector2i pixel_i(std::round(pixel(0)), std::round(pixel(1)));
    // TODO mirror needed?
    Eigen::Vector2i pixel_i(cam.calibration.resolution[0] - std::round(pixel(0)), std::round(pixel(1)));
    cv::Vec3b color_vec = img.at<cv::Vec3b>(pixel_i(0), pixel_i(1));
    color.r = color_vec[0];
    color.g = color_vec[1];
    color.b = color_vec[2];
    return true;
  } else {
    return false;
  }
}

std::string ColorCloudFromImage::intrinsicsToString(const IntrinsicCalibration& calibration) {
  std::stringstream ss;
  ss << "Intrinsic calibration:" << std::endl;
  ss << " -- Camera model: " << calibration.camera_model << std::endl;
  ss << " -- Camera coeffs: " << vecToString(calibration.intrinsics) << std::endl;
  ss << " -- Distortion mode: " << calibration.distortion_model << std::endl;
  ss << " -- Distortion coeffs: " << vecToString(calibration.distortion_coeffs) << std::endl;
  ss << " -- Resolution: " << vecToString(calibration.resolution) << std::endl;
  return ss.str();
}

}
