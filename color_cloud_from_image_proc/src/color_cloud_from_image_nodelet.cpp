#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

//#include "image_undistort/image_undistort.h"

namespace color_cloud {

class ColorCloudFromImageNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      //image_undistort_ = std::make_shared<ImageUndistort>(
      //    getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  //std::shared_ptr<ImageUndistort> image_undistort_;
};
}

PLUGINLIB_DECLARE_CLASS(color_cloud, ColorCloudFromImageNodelet,
                        color_cloud::ColorCloudFromImageNodelet,
                        nodelet::Nodelet);
