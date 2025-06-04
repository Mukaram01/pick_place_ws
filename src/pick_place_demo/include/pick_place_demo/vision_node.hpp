#ifndef PICK_PLACE_DEMO_VISION_NODE_HPP_
#define PICK_PLACE_DEMO_VISION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <string>
#include <vector>

namespace pick_place_demo
{

enum class VisionPipelineType 
{
  SIMPLE_COLOR_DETECTION,
  YOLO_DETECTION,
  SEGMENTATION
};

class VisionNode : public rclcpp::Node
{
public:
  explicit VisionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~VisionNode() = default;

private:
  void camera_callback(
    const sensor_msgs::msg::Image::SharedPtr rgb_msg,
    const sensor_msgs::msg::Image::SharedPtr depth_msg);
  
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  
  void detect_objects(
    const sensor_msgs::msg::Image::SharedPtr & rgb_image,
    const sensor_msgs::msg::Image::SharedPtr & depth_image,
    vision_msgs::msg::Detection3DArray & detections);

  void detect_with_simple_color(
    const sensor_msgs::msg::Image::SharedPtr & rgb_image,
    const sensor_msgs::msg::Image::SharedPtr & depth_image,
    vision_msgs::msg::Detection3DArray & detections);

  void detect_with_yolo(
    const sensor_msgs::msg::Image::SharedPtr & rgb_image,
    const sensor_msgs::msg::Image::SharedPtr & depth_image,
    vision_msgs::msg::Detection3DArray & detections);

  void detect_with_segmentation(
    const sensor_msgs::msg::Image::SharedPtr & rgb_image,
    const sensor_msgs::msg::Image::SharedPtr & depth_image,
    vision_msgs::msg::Detection3DArray & detections);
  
  bool transform_to_world_frame(
    const geometry_msgs::msg::PoseStamped & pose_in,
    geometry_msgs::msg::PoseStamped & pose_out);

  // Parameters
  VisionPipelineType pipeline_type_;
  std::string camera_frame_;
  std::string target_frame_;
  double min_detection_confidence_;
  bool use_sim_time_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  
  // Synchronized message handling
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_filter_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_filter_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  std::shared_ptr<Synchronizer> sync_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Camera info
  bool camera_info_received_;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
};

}  // namespace pick_place_demo

#endif  // PICK_PLACE_DEMO_VISION_NODE_HPP_
