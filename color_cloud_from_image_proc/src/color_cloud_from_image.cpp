#include <color_cloud_from_image_proc/color_cloud_from_image.h>

namespace color_cloud_from_image {

using namespace aslam::cameras;

ColorCloudFromImage::ColorCloudFromImage()
  : nh_(ros::NodeHandle()) {


  cloud_sub_ = nh_.subscribe("cloud", 10, &ColorCloudFromImage::cloudCallback, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("colored_cloud", 1000, false);
}

IntrinsicCalibration ColorCloudFromImage::loadCalibration(const ros::NodeHandle& nh) {

}

void ColorCloudFromImage::addCamera(std::string name, std::string topic, const ros::NodeHandle& nh) {
  addCamera(name, topic, loadCalibration(nh));
}


void ColorCloudFromImage::addCamera(std::string name, std::string topic, const IntrinsicCalibration& calibration) {
  image_transport::ImageTransport it(nh_);

  std::pair<std::string, image_transport::Subscriber> entry(name, it.subscribe(topic, 1, boost::bind(&ColorCloudFromImage::imageCallback, this, name, _1)));
  cam_subs_.emplace(entry);

  RadialTangentialDistortion distortion(calibration.k1, calibration.k2, calibration.p1, calibration.p2);
  OmniProjection<RadialTangentialDistortion> projection(calibration.xi, calibration.fu, calibration.fv, calibration.cu, calibration.cv, calibration.ru, calibration.rv, distortion);

  CameraModel model;
  model.name = name;
  model.camera.reset(new CameraGeometry<OmniProjection<RadialTangentialDistortion>, GlobalShutter, NoMask>(projection));
  std::pair<std::string, CameraModel> entry2(name, model);
  cameras_.emplace(entry2);
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


void ColorCloudFromImage::worldToColor(const Eigen::Vector3d& point3d) {
  bool found_color = false;
  for (std::map<std::string, CameraModel>::iterator c = cameras_.begin(); c != cameras_.end() && !found_color; ++c) {
    boost::shared_ptr<aslam::cameras::CameraGeometryBase> camera = c->second.camera;
    Eigen::VectorXd pixel(2);
    if (camera->vsEuclideanToKeypoint(point3d, pixel)) {
      //TODO somehow get pixel color from last image of cam and return that color
      found_color = true;
    }
  }
}

}
