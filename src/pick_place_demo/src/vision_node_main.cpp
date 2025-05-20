#include "rclcpp/rclcpp.hpp"
#include "pick_place_demo/vision_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pick_place_demo::VisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
