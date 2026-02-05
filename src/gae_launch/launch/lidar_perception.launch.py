import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def launch_setup(context, *args, **kwargs):
    # Determine if debug mode is on
    debug = LaunchConfiguration("debug").perform(context)
    
    # Load parameters for ground_segmentation node
    ground_segmentation_param_path = LaunchConfiguration(
        "ground_segmentation_param_path"    
    ).perform(context)
    with open(ground_segmentation_param_path, "r") as f:
        ground_segmentation_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # Load parameters for object_clustering node
    object_clustering_param_path = LaunchConfiguration(
        "object_clustering_param_path"
    ).perform(context)
    with open(object_clustering_param_path, "r") as f:
        object_clustering_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        
    # Add debug parameter to both nodes
    ground_segmentation_param["debug_mode"] = (debug == "true")
    object_clustering_param["debug_mode"] = (debug == "true")

    # Define the composable nodes
    nodes = [
        ComposableNode(
            package="ground_segmentation",
            plugin="ground_segmentation::TravelGroundFilterComponent",
            name="ground_segmentation_node",
            parameters=[ground_segmentation_param],
        ),
        ComposableNode(
            package="object_clustering",
            plugin="object_clustering::TravelObjectClusteringComponent",
            name="object_clustering_node",
            parameters=[object_clustering_param],
        ),
    ]

    container = ComposableNodeContainer(
        name="lidar_perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # Multi-threaded container
        composable_node_descriptions=nodes,
        output="screen",
    )

    group = GroupAction(
        [
            container,
        ]
    )

    return [group]


def generate_launch_description():
    # Default parameter file paths
    default_ground_segmentation_param_path = os.path.join(
        get_package_share_directory("ground_segmentation"),
        "config/ground_segmentation.param.yaml",
    )

    default_object_clustering_param_path = os.path.join(
        get_package_share_directory("object_clustering"),
        "config/object_clustering.param.yaml",
    )

    # Declare launch arguments
    ground_segmentation_param = DeclareLaunchArgument(
        "ground_segmentation_param_path",
        default_value=default_ground_segmentation_param_path,
        description="Path to config file for ground_segmentation parameters",
    )

    object_clustering_param = DeclareLaunchArgument(
        "object_clustering_param_path",
        default_value=default_object_clustering_param_path,
        description="Path to config file for object_clustering parameters",
    )
    
    # Declare debug argument
    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description="Enable debug mode",
    )

    return launch.LaunchDescription(
        [
            ground_segmentation_param,
            object_clustering_param,
            debug_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
