#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "pick_place_demo/control_node.hpp"

namespace pick_place_demo {
class ControlNodeTestHelper {
public:
  static void setState(ControlNode & node, RobotState state) { node.current_state_ = state; }
  static void setGraspSuccess(ControlNode & node, bool success) { node.grasp_success_ = success; }
  static void runStateMachine(ControlNode & node) { node.execute_state_machine(); }
};
}

using namespace std::chrono_literals;

TEST(ControlNodeStates, GraspFailureReturnsIdle)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<pick_place_demo::ControlNode>();
  pick_place_demo::ControlNodeTestHelper::setState(*node, pick_place_demo::RobotState::GRASPING);
  pick_place_demo::ControlNodeTestHelper::setGraspSuccess(*node, false);
  pick_place_demo::ControlNodeTestHelper::runStateMachine(*node);
  EXPECT_EQ(node->get_current_state(), pick_place_demo::RobotState::IDLE);
  rclcpp::shutdown();
}

TEST(ControlNodeStates, MoveHomeCompletesCycle)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<pick_place_demo::ControlNode>();
  pick_place_demo::ControlNodeTestHelper::setState(*node, pick_place_demo::RobotState::MOVING_TO_HOME);
  pick_place_demo::ControlNodeTestHelper::runStateMachine(*node);
  EXPECT_EQ(node->get_current_state(), pick_place_demo::RobotState::IDLE);
  rclcpp::shutdown();
}

