from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cell_gui',
            executable='cell_gui_node',
            name='cell_gui',
            output='screen'
        )
    ])
