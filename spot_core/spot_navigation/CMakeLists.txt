################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(spot_navigation)
set(CMAKE_CXX_STANDARD 20)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

################################################################################
# Build executables
################################################################################
add_library(filter_pointcloud SHARED src/filter_pointcloud.cpp)
ament_target_dependencies(filter_pointcloud rclcpp tf2_ros sensor_msgs rclcpp_components)
rclcpp_components_register_nodes(filter_pointcloud "spot_navigation::PointcloudFilterComponent")

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch map config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/publish_initial_pose_for_slam.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS filter_pointcloud
  DESTINATION lib
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
