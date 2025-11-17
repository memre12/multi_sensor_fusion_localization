from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Base_link to imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link']
        ),

        # Base_link to lidar_top
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_lidar',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'lidar_top']
        ),

        # Add more static transforms here if needed
    ])