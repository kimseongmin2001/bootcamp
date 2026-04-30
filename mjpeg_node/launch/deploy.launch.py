from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mjpeg_node'), 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package='mjpeg_node',
            executable='mjpeg_node',
            name='mjpeg_node',
            output='screen',
            parameters=[config],
        ),
    ])
