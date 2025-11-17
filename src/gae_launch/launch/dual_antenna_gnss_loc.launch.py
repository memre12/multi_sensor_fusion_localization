from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource,AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os
import xacro
from ament_index_python.packages import get_package_share_directory
 

def generate_launch_description():
    ekf_localizer_package = get_package_share_directory('ekf_localizer')
    gyro_odometer_package = get_package_share_directory('gyro_odometer')
    gnss_poser_package = get_package_share_directory('gnss_poser')
    map_launch_package = get_package_share_directory('gae_launch')
    rviz_package = get_package_share_directory('rviz2')
    rviz_config_file = os.path.join(map_launch_package, "rviz", "localisation.rviz")
    
    ekf_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(ekf_localizer_package, 'launch', 'ekf_localizer.launch.xml')
        )
    )

    gyro_odometer_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(  # Assuming it's a .launch.xml but using Python parser works
            os.path.join(gyro_odometer_package, 'launch', 'gyro_odometer.launch.xml')
        )
    )

    gnss_poser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gnss_poser_package, 'launch', 'gnss_poser.launch.py')
        )
    )

    map_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(map_launch_package, 'launch', 'map.launch.py')
        )
    )

    twist_converter_raw = ExecuteProcess(
    cmd=[
        'python3',
        os.path.join(map_launch_package, 'scripts', 'twist_to_twist_cov.py')
    ],
    output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(rviz_package, rviz_config_file)],
        parameters=[{'use_sim_time': False}],
    )

    xacro_file = os.path.join(get_package_share_directory("gae_launch"), 'urdf', 'sensors.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}
    tf_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
        
    

    return LaunchDescription([
        ekf_launch,
        gyro_odometer_launch,
        gnss_poser_launch,
        map_launch,
        rviz_node,
        twist_converter_raw,
        tf_node
    ])
