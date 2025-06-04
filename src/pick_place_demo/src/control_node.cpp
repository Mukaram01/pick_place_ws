#include "pick_place_demo/control_node.hpp"

namespace pick_place_demo
{

ControlNode::ControlNode(const rclcpp::NodeOptions & options)
: Node("control_node", options),
  current_state_(RobotState::IDLE),
  target_received_(false),
  grasp_success_(false)
{
  // Parameters
  arm_group_name_ = this->declare_parameter<std::string>("arm_group_name", "delta_arm");
  gripper_group_name_ = this->declare_parameter<std::string>("gripper_group_name", "gripper");
  end_effector_link_ = this->declare_parameter<std::string>("end_effector_link", "ee_link");
  gripper_open_value_ = this->declare_parameter<double>("gripper_open_value", 0.04);
  gripper_close_value_ = this->declare_parameter<double>("gripper_close_value", 0.0);
  planning_time_ = this->declare_parameter<double>("planning_time", 5.0);
  pick_approach_distance_ = this->declare_parameter<double>("pick_approach_distance", 0.1);
  place_retreat_distance_ = this->declare_parameter<double>("place_retreat_distance", 0.1);
  gripper_type_ = this->declare_parameter<std::string>("gripper_type", "suction");
  
  // Set fixed place pose
  place_pose_.header.frame_id = "world";
  place_pose_.pose.position.x = this->declare_parameter<double>("place_pose_x", 0.4);
  place_pose_.pose.position.y = this->declare_parameter<double>("place_pose_y", -0.3);
  place_pose_.pose.position.z = this->declare_parameter<double>("place_pose_z", 0.7);
  place_pose_.pose.orientation.w = 1.0;
  
  // Initialize MoveIt interfaces
  // MoveGroupInterface will be serviced by the main executor
  
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), arm_group_name_);
  
  planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  
  // Configure move group
  move_group_->setPlanningTime(planning_time_);
  move_group_->setEndEffectorLink(end_effector_link_);
  
  RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized");
  
  // Setup ROS interfaces
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "target_pose", 10, std::bind(&ControlNode::target_pose_callback, this, std::placeholders::_1));
  
  if (gripper_type_ == "suction") {
    enable_suction_client_ = this->create_client<std_srvs::srv::Trigger>("enable_suction");
  }
  
  // Start state machine timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&ControlNode::timer_callback, this));
    
  RCLCPP_INFO(this->get_logger(), "Control node initialized");
}

void ControlNode::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (current_state_ == RobotState::IDLE) {
    current_target_pose_ = *msg;
    target_received_ = true;
    current_state_ = RobotState::PLANNING;
    
    // Initialize cycle data
    current_cycle_.cycle_start_time = this->now();
    current_cycle_.success = false;
    current_cycle_.planning_time = 0.0;
    current_cycle_.execution_time = 0.0;
    current_cycle_.traveled_distance = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "Received new target pose: [%f, %f, %f]", 
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  } else {
    RCLCPP_WARN(this->get_logger(), "Received target pose while robot is busy, ignoring.");
  }
}

void ControlNode::timer_callback()
{
  execute_state_machine();
}

void ControlNode::execute_state_machine()
{
  switch (current_state_) {
    case RobotState::IDLE:
      // Nothing to do, waiting for target
      break;
    
    case RobotState::PLANNING:
      {
        RCLCPP_INFO(this->get_logger(), "Planning path to pick pose");
        
        // Create approach pose (slightly above object)
        geometry_msgs::msg::PoseStamped approach_pose = current_target_pose_;
        approach_pose.pose.position.z += pick_approach_distance_;
        
        // First plan to approach pose
        move_group_->setPoseTarget(approach_pose.pose);
        
        auto start_time = this->now();
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        auto end_time = this->now();
        
        current_cycle_.planning_time = (end_time - start_time).seconds();
        
        if (success) {
          RCLCPP_INFO(this->get_logger(), "Planning successful, moving to pick approach position");
          move_group_->execute(plan);
          current_state_ = RobotState::MOVING_TO_PICK;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Planning failed, returning to idle");
          current_state_ = RobotState::IDLE;
          current_cycle_.success = false;
          log_cycle_data();
        }
      }
      break;
    
    case RobotState::MOVING_TO_PICK:
      {
        RCLCPP_INFO(this->get_logger(), "Moving down to grasp object");
        
        // Now move down to the actual target
        move_group_->setPoseTarget(current_target_pose_.pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
          move_group_->execute(plan);
          current_state_ = RobotState::GRASPING;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to plan path to grasp position, returning to idle");
          current_state_ = RobotState::IDLE;
          current_cycle_.success = false;
          log_cycle_data();
        }
      }
      break;
    
    case RobotState::GRASPING:
      {
        RCLCPP_INFO(this->get_logger(), "Grasping object");
        
        // Activate the gripper
        if (gripper_type_ == "suction") {
          grasp_success_ = enable_suction(true);
        } else {
          // For finger gripper, we would send a gripper close command here
          // For simulation, we'll just assume success
          grasp_success_ = true;
        }
        
        if (grasp_success_) {
          RCLCPP_INFO(this->get_logger(), "Grasp successful, moving back to approach position");
          
          // Create retreat pose (move straight up)
          geometry_msgs::msg::PoseStamped retreat_pose = current_target_pose_;
          retreat_pose.pose.position.z += pick_approach_distance_;
          
          move_group_->setPoseTarget(retreat_pose.pose);
          
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
          
          if (success) {
            move_group_->execute(plan);
            current_state_ = RobotState::MOVING_TO_PLACE;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan retreat path, returning to idle");
            
            // Release object
            if (gripper_type_ == "suction") {
              enable_suction(false);
            }
            
            current_state_ = RobotState::IDLE;
            current_cycle_.success = false;
            log_cycle_data();
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to grasp object, returning to idle");
          current_state_ = RobotState::IDLE;
          current_cycle_.success = false;
          log_cycle_data();
        }
      }
      break;
    
    case RobotState::MOVING_TO_PLACE:
      {
        RCLCPP_INFO(this->get_logger(), "Planning path to place position");
        
        // Plan to place position
        move_group_->setPoseTarget(place_pose_.pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
          move_group_->execute(plan);
          current_state_ = RobotState::RELEASING;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to plan path to place position, returning to idle");
          
          // Release object
          if (gripper_type_ == "suction") {
            enable_suction(false);
          }
          
          current_state_ = RobotState::IDLE;
          current_cycle_.success = false;
          log_cycle_data();
        }
      }
      break;
    
    case RobotState::RELEASING:
      {
        RCLCPP_INFO(this->get_logger(), "Releasing object");
        
        // Release the gripper
        if (gripper_type_ == "suction") {
          enable_suction(false);
        } else {
          // For finger gripper, we would send a gripper open command here
        }
        
        // Create retreat pose from place
        geometry_msgs::msg::PoseStamped retreat_pose = place_pose_;
        retreat_pose.pose.position.z += place_retreat_distance_;
        
        move_group_->setPoseTarget(retreat_pose.pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
          move_group_->execute(plan);
          current_state_ = RobotState::MOVING_TO_HOME;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to plan retreat from place, returning to idle");
          current_state_ = RobotState::IDLE;
          current_cycle_.success = true;  // Object was still placed
          log_cycle_data();
        }
      }
      break;
    
    case RobotState::MOVING_TO_HOME:
      {
        RCLCPP_INFO(this->get_logger(), "Moving to home position");
        
        // Move to home joint position
        move_group_->setNamedTarget("home");
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
          move_group_->execute(plan);
          
          // Log successful cycle
          current_cycle_.success = true;
          current_cycle_.cycle_end_time = this->now();
          current_cycle_.execution_time = (current_cycle_.cycle_end_time - current_cycle_.cycle_start_time).seconds();
          log_cycle_data();
          
          // Return to idle state
          current_state_ = RobotState::IDLE;
          target_received_ = false;
          
          RCLCPP_INFO(this->get_logger(), "Pick and place cycle completed successfully");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to plan path to home, returning to idle");
          current_state_ = RobotState::IDLE;
          current_cycle_.success = true;  // Object was still placed
          log_cycle_data();
        }
      }
      break;
    
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown state in state machine");
      current_state_ = RobotState::IDLE;
      break;
  }
}

bool ControlNode::enable_suction(bool enable)
{
  if (!enable_suction_client_) {
    RCLCPP_ERROR(this->get_logger(), "Suction client not available");
    return false;
  }
  
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  
  auto future = enable_suction_client_->async_send_request(request);
  
  // Wait for response
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    return response->success;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call suction service");
    return false;
  }
}

void ControlNode::log_cycle_data()
{
  // Add to completed cycles
  completed_cycles_.push_back(current_cycle_);
  
  // Log metrics
  RCLCPP_INFO(this->get_logger(), "Cycle completed: Success=%s, Planning time=%.2fs, Total time=%.2fs",
              current_cycle_.success ? "true" : "false",
              current_cycle_.planning_time,
              current_cycle_.execution_time);
  
  // Calculate success rate
  int success_count = 0;
  for (const auto& cycle : completed_cycles_) {
    if (cycle.success) {
      success_count++;
    }
  }
  
  double success_rate = static_cast<double>(success_count) / completed_cycles_.size() * 100.0;
  double avg_execution_time = 0.0;
  for (const auto& cycle : completed_cycles_) {
    avg_execution_time += cycle.execution_time;
  }
  avg_execution_time /= completed_cycles_.size();
  
  RCLCPP_INFO(this->get_logger(), "Overall metrics: Success rate=%.1f%%, Avg time=%.2fs, Total cycles=%ld",
              success_rate, avg_execution_time, completed_cycles_.size());
}

}  // namespace pick_place_demo

// Register the component with class_loader
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pick_place_demo::ControlNode)
