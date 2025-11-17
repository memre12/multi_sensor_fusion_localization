cmake_minimum_required(VERSION 3.8)
project(gae_pose_initializer_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(component_interface_specs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)


add_executable(gae_pose_initializer_v2 src/gae_pose_initializer_node.cpp)
ament_target_dependencies(gae_pose_initializer_v2 rclcpp geometry_msgs component_interface_specs tf2_ros tf2_geometry_msgs tf2)

install(TARGETS
  gae_pose_initializer_v2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
