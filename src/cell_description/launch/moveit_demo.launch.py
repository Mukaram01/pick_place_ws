import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set package paths
    pkg_share = FindPackageShare(package='cell_description').find('cell_description')
    moveit_config_pkg_share = FindPackageShare(package='pick_place_demo').find('pick_place_demo')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gripper_type = LaunchConfiguration('gripper_type', default='suction')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true')
    
    declare_gripper_type = DeclareLaunchArgument(
        'gripper_type',
        default_value='suction',
        description='Type of gripper: "suction" or "two_finger"')
    
    # Get robot description from URDF
    robot_description_content = Command([
        'xacro ', os.path.join(pkg_share, 'urdf', 'pick_place_cell.urdf.xacro'), 
        ' use_gripper_type:=', gripper_type
    ])
    
    # Get parameters for moveit
    robot_description = {'robot_description': robot_description_content}
    robot_description_semantic = {'robot_description_semantic': 
                                 Command(['cat ', os.path.join(moveit_config_pkg_share, 'config', 'delta_robot.srdf')])}
    
    kinematics_yaml = os.path.join(moveit_config_pkg_share, 'config', 'kinematics.yaml')
    ompl_planning_yaml = os.path.join(moveit_config_pkg_share, 'config', 'ompl_planning.yaml')
    moveit_controllers_yaml = os.path.join(moveit_config_pkg_share, 'config', 'controllers.yaml')
    
    # Start move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_yaml,
            moveit_controllers_yaml,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Start RViz with MoveIt configuration
    rviz_config_file = os.path.join(moveit_config_pkg_share, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_gripper_type,
        move_group_node,
        rviz_node
    ])
