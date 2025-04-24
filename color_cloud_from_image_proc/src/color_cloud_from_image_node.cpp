#include <color_cloud_from_image_proc/color_cloud_from_image.h>

namespace color_cloud_from_image {

class ColorCloudFromImageNode {
 public:
  ColorCloudFromImageNode(const rclcpp::NodeOptions& options)
    : Node("color_cloud_from_image_node", options) {
    color_cloud_ = std::make_shared<ColorCloudFromImage>(this->shared_from_this());
  }
 private:
  rclcpp::Node::SharedPtr node_;
  boost::shared_ptr<ColorCloudFromImage> color_cloud_;
};
}
