from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg       = get_package_share_directory('satellite_node')
    config    = os.path.join(pkg, 'config',  'config.yaml')
    rviz_cfg  = os.path.join(pkg, 'rviz',    'satellite.rviz')
    mesh_path = os.path.join(pkg, 'meshes',  'joby_s4.stl')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='RViz 실행 여부')

    # RViz Fixed Frame "map" 을 TF 트리에 등록
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen',
    )

    satellite = Node(
        package='satellite_node',
        executable='satellite_node',
        name='satellite_node',
        output='screen',
        parameters=[config, {'mesh_path': mesh_path}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_rviz_arg,
        static_tf,
        satellite,
        rviz,
    ])
