#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>

#include "pick_place_demo/control_node.hpp"

using namespace std::chrono_literals;

TEST(ControlNodeTest, TargetPoseAdvancesState)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<pick_place_demo::ControlNode>();

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);

  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "world";
  msg.pose.orientation.w = 1.0;

  EXPECT_EQ(node->get_current_state(), pick_place_demo::RobotState::IDLE);

  executor->spin_some();
  pub->publish(msg);

  for (int i = 0; i < 5; ++i) {
    executor->spin_some();
    rclcpp::sleep_for(100ms);
  }

  EXPECT_EQ(node->get_current_state(), pick_place_demo::RobotState::PLANNING);

  rclcpp::shutdown();
}

