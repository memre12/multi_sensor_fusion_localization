cmake_minimum_required(VERSION 3.14)
project(gnss_poser)

## Set up build tool dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(autoware_cmake REQUIRED)
autoware_package()
## Find other dependencies
find_package(geographic_msgs REQUIRED)
find_package(geography_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)
find_package(message_filters REQUIRED)


## Define the headers for your package
set(GNSS_POSER_HEADERS
  include/gnss_poser/gnss_poser_core.hpp
)
## Add the library
ament_auto_add_library(gnss_poser_node SHARED
  src/gnss_poser_core.cpp
  ${GNSS_POSER_HEADERS}
)

## Link the necessary libraries
target_link_libraries(gnss_poser_node Geographic)

## Register the ROS2 components
rclcpp_components_register_node(gnss_poser_node
  PLUGIN "gnss_poser::GNSSPoser"
  EXECUTABLE gnss_poser
)

## Define the dependencies
ament_target_dependencies(gnss_poser_node
  rclcpp
  rclcpp_components
  geographic_msgs
  geography_utils
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  Boost
  message_filters
)

## Install the package
ament_auto_package(INSTALL_TO_SHARE
  config
  launch
  include
)
