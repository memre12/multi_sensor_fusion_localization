from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'urdf',
        'sensors.urdf'  # Make sure this matches the actual filename
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command([urdf_file_path])}]
        )
    ])
