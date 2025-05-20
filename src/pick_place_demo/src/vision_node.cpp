#include "pick_place_demo/vision_node.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace pick_place_demo
{

VisionNode::VisionNode(const rclcpp::NodeOptions & options)
: Node("vision_node", options),
  camera_info_received_(false)
{
  // Get parameters
  std::string pipeline_type_str = this->declare_parameter<std::string>("pipeline_type", "simple_color");
  if (pipeline_type_str == "simple_color") {
    pipeline_type_ = VisionPipelineType::SIMPLE_COLOR_DETECTION;
  } else if (pipeline_type_str == "yolo") {
    pipeline_type_ = VisionPipelineType::YOLO_DETECTION;
  } else if (pipeline_type_str == "segmentation") {
    pipeline_type_ = VisionPipelineType::SEGMENTATION;
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown pipeline type '%s', defaulting to simple color detection", 
                pipeline_type_str.c_str());
    pipeline_type_ = VisionPipelineType::SIMPLE_COLOR_DETECTION;
  }
  
  camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
  target_frame_ = this->declare_parameter<std::string>("target_frame", "world");
  min_detection_confidence_ = this->declare_parameter<double>("min_detection_confidence", 0.5);
  use_sim_time_ = this->declare_parameter<bool>("use_sim_time", true);
  
  // Setup TF listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Setup publishers
  detections_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("detected_objects", 10);
  target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
  
  // Setup message filters for synchronized RGB and depth images
  rgb_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "/camera/color/image_raw");
  depth_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "/camera/depth/image_rect_raw");
  
  sync_ = std::make_shared<Synchronizer>(
    SyncPolicy(10), *rgb_filter_sub_, *depth_filter_sub_);
  sync_->registerCallback(std::bind(&VisionNode::camera_callback, this, 
                                    std::placeholders::_1, std::placeholders::_2));
  
  // Setup camera info subscription
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", 10, 
    std::bind(&VisionNode::camera_info_callback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), "Vision node initialized with %s pipeline", pipeline_type_str.c_str());
}

void VisionNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  camera_info_ = msg;
  camera_info_received_ = true;
  
  // We only need one camera info message
  camera_info_sub_.reset();
  
  RCLCPP_INFO(this->get_logger(), "Received camera info");
}

void VisionNode::camera_callback(
  const sensor_msgs::msg::Image::SharedPtr rgb_msg,
  const sensor_msgs::msg::Image::SharedPtr depth_msg)
{
  if (!camera_info_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                         "Waiting for camera info...");
    return;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Received synchronized RGB and depth images");
  
  // Detect objects in the scene
  vision_msgs::msg::Detection3DArray detections;
  detections.header = rgb_msg->header;
  
  try {
    detect_objects(rgb_msg, depth_msg, detections);
    
    // Publish all detections
    if (!detections.detections.empty()) {
      detections_pub_->publish(detections);
      
      // Publish the first detection as target pose
      if (!detections.detections.empty()) {
        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.header = detections.header;
        pose_in.pose = detections.detections[0].results[0].pose.pose;
        
        if (transform_to_world_frame(pose_in, pose_out)) {
          target_pose_pub_->publish(pose_out);
        }
      }
    }
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing images: %s", e.what());
  }
}

void VisionNode::detect_objects(
  const sensor_msgs::msg::Image::SharedPtr & rgb_image,
  const sensor_msgs::msg::Image::SharedPtr & depth_image,
  vision_msgs::msg::Detection3DArray & detections)
{
  // Convert RGB image to OpenCV format
  cv_bridge::CvImagePtr cv_rgb;
  try {
    cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  
  // Convert depth image to OpenCV format
  cv_bridge::CvImagePtr cv_depth;
  try {
    cv_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  
  // For this simulation we'll use a simple color-based detection
  // In a real application you would use more sophisticated algorithms or YOLO/segmentation
  
  // Simple red object detection as an example
  cv::Mat hsv;
  cv::cvtColor(cv_rgb->image, hsv, cv::COLOR_BGR2HSV);
  
  // Define range for red color detection
  cv::Mat mask1, mask2;
  cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
  cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), mask2);
  
  // Combine masks for full red range (wraps at both ends of hue range)
  cv::Mat mask = mask1 | mask2;
  
  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
  // Process contours
  for (size_t i = 0; i < contours.size(); i++) {
    // Filter small contours
    if (cv::contourArea(contours[i]) < 100) continue;
    
    // Find the bounding rectangle
    cv::Rect bounding_rect = cv::boundingRect(contours[i]);
    
    // Get center point in image coordinates
    int cx = bounding_rect.x + bounding_rect.width / 2;
    int cy = bounding_rect.y + bounding_rect.height / 2;
    
    // Get depth at center point (convert from mm to m)
    uint16_t depth_value = cv_depth->image.at<uint16_t>(cv::Point(cx, cy));
    double depth_meters = static_cast<double>(depth_value) / 1000.0;
    
    // Skip if depth is invalid
    if (depth_meters <= 0.01 || depth_meters > 5.0) continue;
    
    // Convert pixel coordinates to 3D point
    // This assumes the depth and RGB images are registered and have the same resolution
    
    // Get camera intrinsics
    double fx = camera_info_->k[0];
    double fy = camera_info_->k[4];
    double cx_cam = camera_info_->k[2];
    double cy_cam = camera_info_->k[5];
    
    // Convert to camera coordinates
    double x = (cx - cx_cam) * depth_meters / fx;
    double y = (cy - cy_cam) * depth_meters / fy;
    double z = depth_meters;
    
    // Create detection result
    vision_msgs::msg::Detection3D detection;
    detection.header = rgb_image->header;
    
    // Set the 3D bounding box (just a small cube for now)
    detection.bbox.center.position.x = x;
    detection.bbox.center.position.y = y;
    detection.bbox.center.position.z = z;
    detection.bbox.center.orientation.w = 1.0;  // Identity quaternion
    
    detection.bbox.size.x = 0.05;  // Approximate size in meters
    detection.bbox.size.y = 0.05;
    detection.bbox.size.z = 0.05;
    
    // Add hypothesis
    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = "red_cube";
    hypothesis.hypothesis.score = 0.9;  // High confidence for simulation
    
    // Set the pose for grasping (center of the object)
    hypothesis.pose.pose.position.x = x;
    hypothesis.pose.pose.position.y = y;
    hypothesis.pose.pose.position.z = z;
    hypothesis.pose.pose.orientation.w = 1.0;  // Identity quaternion
    
    detection.results.push_back(hypothesis);
    
    // Add to detections array
    detections.detections.push_back(detection);
    
    // Draw detection for visualization
    cv::rectangle(cv_rgb->image, bounding_rect, cv::Scalar(0, 255, 0), 2);
    cv::circle(cv_rgb->image, cv::Point(cx, cy), 3, cv::Scalar(0, 0, 255), -1);
    
    RCLCPP_INFO(this->get_logger(), "Detected object at 3D position: (%f, %f, %f)", x, y, z);
  }
  
  // Publish the annotated image (optional for debug)
  // debug_image_pub_->publish(cv_rgb->toImageMsg());
}

bool VisionNode::transform_to_world_frame(
  const geometry_msgs::msg::PoseStamped & pose_in,
  geometry_msgs::msg::PoseStamped & pose_out)
{
  try {
    tf_buffer_->transform(pose_in, pose_out, target_frame_);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    return false;
  }
}

}  // namespace pick_place_demo

// Register the component with class_loader
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pick_place_demo::VisionNode)
