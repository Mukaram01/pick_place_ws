import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_type",
            default_value="suction",
            description="Type of gripper to use (suction, two_finger)",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Use Gazebo simulation or real hardware",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "vision_pipeline",
            default_value="simple_color",
            description="Vision pipeline to use (simple_color, yolo, segmentation)",
        )
    )
    
    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    gripper_type = LaunchConfiguration("gripper_type")
    use_sim = LaunchConfiguration("use_sim")
    vision_pipeline = LaunchConfiguration("vision_pipeline")
    
    # Find package paths
    pkg_pick_place_demo = FindPackageShare("pick_place_demo").find("pick_place_demo")
    pkg_cell_description = FindPackageShare("cell_description").find("cell_description")
    
    # Include simulation launch if using sim
    sim_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cell_description, "launch", "gazebo_simulation.launch.py")
        ),
        condition=IfCondition(use_sim),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gripper_type": gripper_type,
        }.items(),
    )
    
    # Include RealSense launch if using real hardware
    realsense_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("realsense2_camera").find("realsense2_camera"), 
                        "launch", "rs_launch.py")
        ),
        condition=UnlessCondition(use_sim),
        launch_arguments={
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_infra1": "false",
            "enable_infra2": "false",
        }.items(),
    )
    
    # Include MoveIt launch
    moveit_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pick_place_demo, "launch", "moveit_demo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gripper_type": gripper_type,
        }.items(),
    )
    
    # Create nodes
    vision_node = Node(
        package="pick_place_demo",
        executable="vision_node",
        name="vision_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"pipeline_type": vision_pipeline},
            {"camera_frame": "camera_color_optical_frame"},
            {"target_frame": "world"},
            {"min_detection_confidence": 0.5},
        ],
    )
    
    control_node = Node(
        package="pick_place_demo",
        executable="control_node",
        name="control_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gripper_type": gripper_type},
            {"arm_group_name": "delta_arm"},
            {"gripper_group_name": "gripper"},
            {"end_effector_link": "ee_link"},
            {"place_pose_x": 0.4},
            {"place_pose_y": -0.3},
            {"place_pose_z": 0.85},
        ],
    )
    
    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)
    
    # Add simulation or hardware nodes
    ld.add_action(sim_launch_include)
    ld.add_action(realsense_launch_include)
    
    # Add MoveIt nodes
    ld.add_action(moveit_launch_include)
    
    # Add our demo nodes
    ld.add_action(vision_node)
    ld.add_action(control_node)
    
    return ld
