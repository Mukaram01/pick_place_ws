# ROS 2 Vision-Guided Pick-and-Place Cell

A complete vision-guided pick-and-place system built in ROS 2 Humble, featuring a delta robot, conveyor belt, and various grippers for automated object picking and placing.

![System Overview](docs/images/system_overview.png)

## Features

- **Modular Design**: Three packages for robot description, control, and GUI
- **Robot Options**: Delta robot with customizable parameters
- **End-Effector Options**: 
  - Suction gripper for flat objects
  - Two-finger gripper for various shaped objects
- **Conveyor System**: Customizable size and speed
- **Vision Integration**:
  - Intel RealSense D435i support
  - Multiple vision pipeline options (color detection, YOLO, segmentation)
- **Motion Planning**: Full MoveIt 2 integration
- **User Interface**: Qt-based GUI for configuration and control
- **Performance Monitoring**: Real-time cycle time and success rate tracking
- **Simulation**: Full Gazebo integration for testing without hardware

## System Requirements

- **OS**: Ubuntu 22.04
- **ROS**: ROS 2 Humble
- **3D Simulation**: Gazebo 11
- **Motion Planning**: MoveIt 2
- **GUI**: Qt 5
- **Camera** (for hardware deployment): Intel RealSense D435i
- **Target Hardware** (optional): NVIDIA Jetson Nano/Orin

## Installation

### 1. Install ROS 2 Humble

Follow the [official ROS 2 installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

### 2. Install Dependencies

```bash
# ROS and Gazebo dependencies
sudo apt update
sudo apt install -y \
  python3-pip \
  python3-colcon-common-extensions \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description \
  ros-humble-vision-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2

# Qt dependencies
sudo apt install -y \
  qtcreator \
  qt5-default \
  libqt5widgets5

# Python dependencies
pip3 install numpy opencv-python open3d pyrealsense2 torch torchvision onnx onnxruntime

## License

This project is licensed under the [Apache License 2.0](LICENSE).
