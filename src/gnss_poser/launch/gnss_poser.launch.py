import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('gnss_poser'),
        'config',
        'gnss_poser.param.yaml'
    )

    gnss_poser_node = Node(
        package='gnss_poser',
        executable='gnss_poser',
        name='gnss_poser',
        output='screen',
        parameters=[config_file])
    return LaunchDescription([
        gnss_poser_node
    ])

