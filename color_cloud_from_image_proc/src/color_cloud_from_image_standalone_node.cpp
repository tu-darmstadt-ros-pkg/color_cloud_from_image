#include "color_cloud_from_image_proc/color_cloud_from_image.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto color_cloud_from_image = std::make_shared<color_cloud_from_image::ColorCloudFromImage>(rclcpp::NodeOptions());
  auto node = color_cloud_from_image->get_node_base_interface();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("color_cloud_from_image_node"), "Started " << node->get_name() << " standalone node.");
  //nodelet.load(nodelet_name, "color_cloud_from_image/ColorCloudFromImageNodelet", remap, nargv);
  rclcpp::spin(node);
  return 0;
}
