from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('satellite_node'), 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package='satellite_node',
            executable='satellite_node',
            name='satellite_node',
            output='screen',
            parameters=[config],
        ),
    ])
