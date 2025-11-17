from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'gae_launch'

    rviz_config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'localization.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )
    ])
