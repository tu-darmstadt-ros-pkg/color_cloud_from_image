#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <color_cloud_from_image_proc/color_cloud_from_image.h>

namespace color_cloud_from_image {

class ColorCloudFromImageNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    color_cloud_.reset(new ColorCloudFromImage());
    ros::NodeHandle nh("~");
    color_cloud_->loadCamerasFromNamespace(nh);
  }

  boost::shared_ptr<ColorCloudFromImage> color_cloud_;
};
}

PLUGINLIB_DECLARE_CLASS(color_cloud_from_image, ColorCloudFromImageNodelet,
                        color_cloud_from_image::ColorCloudFromImageNodelet,
                        nodelet::Nodelet);
