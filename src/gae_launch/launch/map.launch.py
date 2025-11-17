import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'map',
        'map_config.yaml'
    )

    config = yaml.safe_load(open(config_file))

    pointcloud_map_loader_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'map',
        'pointcloud_map_loader.param.yaml'
    )

    pointcloud_map_path_arg = DeclareLaunchArgument(
        'pointcloud_map_path',
        default_value=config['pointcloud_map_path'],
        description='Path to the pointcloud map file'
    )
    
    pointcloud_map_metadata_path_arg = DeclareLaunchArgument(
        'pointcloud_map_metadata_path',
        default_value=config['pointcloud_map_metadata_path'],
        description='Path to the pointcloud map metadata file'
    )

    pcd_paths_or_directory_arg = DeclareLaunchArgument(
        'pcd_paths_or_directory',
        default_value= f"[{config['pointcloud_map_path']}]",
        description='PCD paths or directory'
    )

    pcd_metadata_path_arg = DeclareLaunchArgument(
        'pcd_metadata_path',
        default_value=config['pointcloud_map_metadata_path'],
        description='Path to the PCD metadata file'
    )

    output_pointcloud_map_topic_arg = DeclareLaunchArgument(
        'output_pointcloud_map_topic',
        default_value='/map/pointcloud_map',
        description='Output pointcloud map topic name'
    )
    
    output_differential_pointcloud_map_service_arg = DeclareLaunchArgument(
        'output_differential_pointcloud_map_service',
        default_value='/map/get_differential_pointcloud_map',
        description='Output differential pointcloud map service name'
    )

    pointcloud_map_loader_node = Node(
        package='map_loader',
        executable='pointcloud_map_loader',
        name='pointcloud_map_loader',
        output='both',
        parameters=[
            pointcloud_map_loader_param_path,
            {'pointcloud_map_path': LaunchConfiguration('pointcloud_map_path')},
            {'pointcloud_map_metadata_path': LaunchConfiguration('pointcloud_map_metadata_path')},
            {'pcd_paths_or_directory': LaunchConfiguration('pcd_paths_or_directory')},
            {'pcd_metadata_path': LaunchConfiguration('pcd_metadata_path')},
        ],
        remappings=[
            ('output/pointcloud_map', LaunchConfiguration('output_pointcloud_map_topic')),
            ('/service/get_differential_pcd_map', LaunchConfiguration('output_differential_pointcloud_map_service'))
        ]
    )


    map_projection_loader_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'map',
        'map_projection_loader.param.yaml'
    )

    map_projector_info_path_arg = DeclareLaunchArgument(
        'map_projector_info_path',
        default_value=config['map_projector_info_path'],
        description='Path to the map projector info file'
    )

    lanelet2_map_path_arg = DeclareLaunchArgument(
        'lanelet2_map_path',
        default_value=config['lanelet2_map_path'],
        description='Path to the lanelet2 map file'
    )

    map_projection_loader_node = Node(
        package='map_projection_loader',
        executable='map_projection_loader_node',
        name='map_projection_loader',
        output='both',
        parameters=[
            map_projection_loader_param_path,
            {'map_projector_info_path': LaunchConfiguration('map_projector_info_path')},
            {'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path')},
        ]
    )


    lanelet2_map_loader_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'map',
        'lanelet2_map_loader.param.yaml'
    )
    lanelet2_map_topic_arg = DeclareLaunchArgument(
        'lanelet2_map_topic',
        default_value='/map/lanelet2_map',
        description='Lanelet2 map topic name'
    )

    lanelet2_map_loader_node = Node(
        package='map_loader',
        executable='lanelet2_map_loader',
        name='lanelet2_map_loader',
        output='both',
        parameters=[
            lanelet2_map_loader_param_path,
            {'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path')},
        ],
        remappings=[
            ('output/lanelet2_map', LaunchConfiguration('lanelet2_map_topic'))
        ]
    )


    lanelet2_map_marker_topic_arg = DeclareLaunchArgument(
        'lanelet2_map_marker_topic',
        default_value='/map/lanelet2_map_marker',
        description='Lanelet2 map marker topic name'
    )

    map_hash_generator_node = Node(
        package='map_loader',
        executable='map_hash_generator',
        name='map_hash_generator',
        output='both',
        parameters=[
            lanelet2_map_loader_param_path
        ]
    )


    lanelet2_map_visualization_node = Node(
        package='map_loader',
        executable='lanelet2_map_visualization',
        name='lanelet2_map_visualization',
        output='both',
        parameters=[
            lanelet2_map_loader_param_path,
            {'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path')},
        ],
        remappings=[
            ('input/lanelet2_map', LaunchConfiguration('lanelet2_map_topic')),
            ('output/lanelet2_map_marker', LaunchConfiguration('lanelet2_map_marker_topic'))
        ]
    )


    map_tf_generator_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'map',
        'map_tf_generator.param.yaml'
    )

    get_map_hash_info_topic_arg = DeclareLaunchArgument(
        'get_map_hash_info_topic',
        default_value='/map/hash',
        description='Map hash info topic name'
    )
    map_tf_generator_node = Node(
        package='map_tf_generator',
        executable='vector_map_tf_generator',
        name='vector_map_tf_generator_node',
        output='both',
        parameters=[
            map_tf_generator_param_path
        ],
        remappings=[
            ('vector_map', LaunchConfiguration('lanelet2_map_topic')),
            ('/api/autoware/get/map/info/hash', LaunchConfiguration('get_map_hash_info_topic'))
        ]
    )

    return LaunchDescription([
        pointcloud_map_path_arg,
        pointcloud_map_metadata_path_arg,
        pcd_paths_or_directory_arg,
        pcd_metadata_path_arg,
        output_pointcloud_map_topic_arg,
        output_differential_pointcloud_map_service_arg,
        pointcloud_map_loader_node,
        map_projector_info_path_arg,
        lanelet2_map_path_arg,
        map_projection_loader_node,
        lanelet2_map_topic_arg,
        lanelet2_map_loader_node,
        lanelet2_map_marker_topic_arg,
        map_hash_generator_node,
        lanelet2_map_visualization_node,
        get_map_hash_info_topic_arg,
        map_tf_generator_node
    ])