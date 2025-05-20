#ifndef PICK_PLACE_DEMO_CONTROL_NODE_HPP_
#define PICK_PLACE_DEMO_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <string>

namespace pick_place_demo
{

enum class RobotState
{
  IDLE,
  PLANNING,
  MOVING_TO_PICK,
  GRASPING,
  MOVING_TO_PLACE,
  RELEASING,
  MOVING_TO_HOME
};

class ControlNode : public rclcpp::Node
{
public:
  explicit ControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~ControlNode() = default;

private:
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void timer_callback();
  bool enable_suction(bool enable);
  void log_cycle_data();
  void execute_state_machine();

  // MoveIt interfaces
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
  
  // State machine variables
  RobotState current_state_;
  geometry_msgs::msg::PoseStamped current_target_pose_;
  geometry_msgs::msg::PoseStamped place_pose_;
  bool target_received_;
  bool grasp_success_;
  
  // Statistics and logging
  struct CycleData {
    double planning_time;
    double execution_time;
    bool success;
    double traveled_distance;
    rclcpp::Time cycle_start_time;
    rclcpp::Time cycle_end_time;
  };
  CycleData current_cycle_;
  std::vector<CycleData> completed_cycles_;
  
  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enable_suction_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Parameters
  std::string arm_group_name_;
  std::string gripper_group_name_;
  std::string end_effector_link_;
  double gripper_open_value_;
  double gripper_close_value_;
  double planning_time_;
  double pick_approach_distance_;
  double place_retreat_distance_;
  std::string gripper_type_;
};

}  // namespace pick_place_demo

#endif  // PICK_PLACE_DEMO_CONTROL_NODE_HPP_
