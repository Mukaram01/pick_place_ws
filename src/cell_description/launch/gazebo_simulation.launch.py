import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set package paths
    pkg_share = FindPackageShare(package='cell_description').find('cell_description')
    gazebo_launch = PythonLaunchDescriptionSource(os.path.join(
        FindPackageShare('gazebo_ros').find('gazebo_ros'), 'launch', 'gazebo.launch.py'))
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gripper_type = LaunchConfiguration('gripper_type', default='suction')
    belt_length = LaunchConfiguration('belt_length', default='2.0')
    belt_width = LaunchConfiguration('belt_width', default='0.5')
    belt_height = LaunchConfiguration('belt_height', default='0.6')
    belt_speed = LaunchConfiguration('belt_speed', default='0.2')
    world_file = LaunchConfiguration('world_file', default=os.path.join(pkg_share, 'worlds', 'pick_place_world.world'))
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true')
    
    declare_gripper_type = DeclareLaunchArgument(
        'gripper_type',
        default_value='suction',
        description='Type of gripper: "suction" or "two_finger"')
    
    declare_belt_length = DeclareLaunchArgument(
        'belt_length',
        default_value='2.0',
        description='Length of the conveyor belt')
    
    declare_belt_width = DeclareLaunchArgument(
        'belt_width',
        default_value='0.5',
        description='Width of the conveyor belt')
    
    declare_belt_height = DeclareLaunchArgument(
        'belt_height',
        default_value='0.6',
        description='Height of the conveyor belt')
    
    declare_belt_speed = DeclareLaunchArgument(
        'belt_speed',
        default_value='0.2',
        description='Speed of the conveyor belt')
    
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_share, 'worlds', 'pick_place_world.world'),
        description='Path to world file')
        
    # Create robot state publisher node
    robot_description_content = Command([
        'xacro ', os.path.join(pkg_share, 'urdf', 'pick_place_cell.urdf.xacro'), 
        ' use_gripper_type:=', gripper_type,
        ' belt_length:=', belt_length,
        ' belt_width:=', belt_width,
        ' belt_height:=', belt_height,
        ' belt_speed:=', belt_speed
    ])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )
    
    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'pick_place_cell'
        ],
        output='screen'
    )
    
    # Load controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )
    
    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", gripper_type, "' == 'two_finger'"]))
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_gripper_type,
        declare_belt_length,
        declare_belt_width,
        declare_belt_height,
        declare_belt_speed,
        declare_world_file,
        IncludeLaunchDescription(
            gazebo_launch,
            launch_arguments={'world': world_file}.items()
        ),
        robot_state_publisher_node,
        spawn_entity,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller
    ])
