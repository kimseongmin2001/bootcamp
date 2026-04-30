from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('udp_node'), 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package='udp_node',
            executable='udp_node',
            name='udp_node',
            output='screen',
            parameters=[config],
        ),
    ])
