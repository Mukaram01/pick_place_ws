#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "pick_place_demo/vision_node.hpp"

class TestableVisionNode : public pick_place_demo::VisionNode
{
public:
  using pick_place_demo::VisionNode::VisionNode;

  void setCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr & msg)
  {
    camera_info_callback(msg);
  }

  void runDetection(const sensor_msgs::msg::Image::SharedPtr & rgb,
                    const sensor_msgs::msg::Image::SharedPtr & depth,
                    vision_msgs::msg::Detection3DArray & output)
  {
    detect_objects(rgb, depth, output);
  }

  bool transformPose(const geometry_msgs::msg::PoseStamped & in,
                     geometry_msgs::msg::PoseStamped & out)
  {
    return transform_to_world_frame(in, out);
  }

  void addTransform(const geometry_msgs::msg::TransformStamped & t)
  {
    tf_buffer_->setTransform(t, "test");
  }
};

TEST(VisionNodeTest, DetectsRedObject)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<TestableVisionNode>();

  auto info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  info->k = {100.0, 0.0, 320.0,
             0.0, 100.0, 240.0,
             0.0, 0.0, 1.0};
  node->setCameraInfo(info);

  cv::Mat rgb(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::rectangle(rgb, cv::Point(200, 200), cv::Point(300, 300), cv::Scalar(0, 0, 255), -1);
  auto rgb_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb).toImageMsg();

  cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(1000));
  auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth).toImageMsg();

  vision_msgs::msg::Detection3DArray detections;
  node->runDetection(rgb_msg, depth_msg, detections);

  EXPECT_GT(detections.detections.size(), 0u);
  rclcpp::shutdown();
}

TEST(VisionNodeTest, TransformPose)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<TestableVisionNode>();

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "camera_color_optical_frame";
  t.child_frame_id = "world";
  t.transform.rotation.w = 1.0;
  node->addTransform(t);

  geometry_msgs::msg::PoseStamped in, out;
  in.header.frame_id = "camera_color_optical_frame";
  in.pose.orientation.w = 1.0;

  EXPECT_TRUE(node->transformPose(in, out));
  EXPECT_EQ(out.header.frame_id, "world");
  rclcpp::shutdown();
}

