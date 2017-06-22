#include <color_cloud_from_image_proc/color_cloud_from_image.h>

#include <cv_bridge/cv_bridge.h>

namespace color_cloud_from_image {

using namespace aslam::cameras;

ColorCloudFromImage::ColorCloudFromImage() {
  nh_ = ros::NodeHandle();

  it_.reset(new image_transport::ImageTransport(nh_));
  tf2_ros::Buffer buffer;
  tf_listener_.reset(new tf2_ros::TransformListener(buffer));


  cloud_sub_ = nh_.subscribe("cloud", 10, &ColorCloudFromImage::cloudCallback, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("colored_cloud", 1000, false);
}

void ColorCloudFromImage::loadCamerasFromNamespace(ros::NodeHandle& nh) {
  XmlRpc::XmlRpcValue cams;
  nh.getParam("~", cams);
  ROS_ASSERT(cams.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = cams.begin(); it != cams.end(); ++it) {
    ROS_INFO_STREAM("Found cam: " << (std::string)(it->first) << " ==> " << cams[it->first]);
    std::string cam_name = (std::string)(it->first);
    std::string rostopic = (std::string)cams[it->first]["rostopic"];
    ros::NodeHandle cam_nh(nh, cam_name);
    IntrinsicCalibration calibration = loadCalibration(cam_nh);
    addCamera(cam_name, rostopic, calibration);
  }
}

IntrinsicCalibration ColorCloudFromImage::loadCalibration(ros::NodeHandle& nh) {
  IntrinsicCalibration calibration;
  calibration.camera_model = nh.param<std::string>("camera_model", "none");
  calibration.intrinsics = nh.param<std::vector<double>>("intrinsics", std::vector<double>(5, 0));
  calibration.distortion_model = nh.param<std::string>("distortion_model", "none");
  calibration.distortion = nh.param<std::vector<double>>("distortion_coeffs", std::vector<double>(4, 0));
  std::vector<int> resolution = nh.param<std::vector<int>>("resolution", std::vector<int>(2, 0));
  if (resolution.size() != 2) {
    ROS_WARN_STREAM("Resolution didn't have size 2");
  } else {
    calibration.resolution.first = resolution[0];
    calibration.resolution.second = resolution[1];
  }
  return calibration;
}



void ColorCloudFromImage::addCamera(std::string name, std::string topic, const IntrinsicCalibration& calibration) {
  Camera cam;
  cam.name = name;

  RadialTangentialDistortion distortion(calibration.distortion[0], calibration.distortion[1],
                                        calibration.distortion[2], calibration.distortion[3]);
  OmniProjection<RadialTangentialDistortion> projection(calibration.intrinsics[0], calibration.intrinsics[1], calibration.intrinsics[2],
                                                        calibration.intrinsics[3], calibration.intrinsics[4], calibration.intrinsics[5],
                                                        calibration.intrinsics[6], distortion);
  cam.camera_model.reset(new CameraGeometry<OmniProjection<RadialTangentialDistortion>, GlobalShutter, NoMask>(projection));

  cam.sub = it_->subscribe(topic, 1, boost::bind(&ColorCloudFromImage::imageCallback, this, name, _1));
  std::pair<std::string, Camera> entry(name, cam);
  cameras_.emplace(entry);
}


void ColorCloudFromImage::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr) {

}

void ColorCloudFromImage::imageCallback(std::string cam_name, const sensor_msgs::ImageConstPtr& image_ptr) {
  try {
    cameras_.at(cam_name).last_image = image_ptr;
  } catch (std::out_of_range) {
    ROS_ERROR_STREAM("Could not find cam " << cam_name);
  }
}


Color ColorCloudFromImage::worldToColor(const Eigen::Vector3d& point3d) {
  for (std::map<std::string, Camera>::iterator c = cameras_.begin(); c != cameras_.end(); ++c) {
    boost::shared_ptr<aslam::cameras::CameraGeometryBase> camera = c->second.camera_model;
    Eigen::VectorXd pixel(2);
    if (camera->vsEuclideanToKeypoint(point3d, pixel)) {
      cv_bridge::CvImageConstPtr cv_image;
      try
      {
        const Camera& cam = c->second;
        const sensor_msgs::ImageConstPtr& img_msg = cam.last_image;
        cv_image = cv_bridge::toCvShare(img_msg);
      }
      catch(cv_bridge::Exception& e)
      {
        ROS_ERROR_STREAM("Conversion failed: " << e.what());
        return Color();
      }
      const cv::Mat& img = cv_image->image;
      Eigen::Vector2i pixel_i(std::round(pixel(0)), std::round(pixel(1)));
      if (pixel_i(0) < 0 || pixel_i(0) >= img.rows || pixel_i(1) << 0 || pixel_i(1) >= img.cols) {
        ROS_WARN_STREAM("Computed pixel out of range");
        continue;
      }
      Color color;
      color.r = img.at<double>(pixel_i(0), pixel_i(1), 0);
      color.g = img.at<double>(pixel_i(0), pixel_i(1), 1);
      color.b = img.at<double>(pixel_i(0), pixel_i(1), 2);
      return color;
    }
  }
  // No color for this pixel
  return Color();
}

}
