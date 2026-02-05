from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():

    velodyne = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('velodyne'),
                'launch',
                'velodyne-all-nodes-VLP16-launch.py'
            ])
        ])
    )
    
    sbg_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sbg_driver'),
                'launch',
                'sb2.launch.py'
            ])
        ])
    )

    zed_node = ExecuteProcess(
        cmd=['python3', 'install/gae_launch/share/gae_launch/launch/zed_minimal_driver.py'],
        output='screen'
    )

    tf_static = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gae_launch'),
                'launch',
                'robot.launch.py'
            ])
        ])
    )

    rtk_rtcm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rbf_ntrip_driver'),
                'launch',
                'rbf_ntrip_driver.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        velodyne,
        sbg_driver,
        zed_node,  
        tf_static,
        rtk_rtcm,
    ])