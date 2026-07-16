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
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(spot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(TBB REQUIRED)

include_directories(
  include
)

################################################################################
# Build executables
################################################################################
add_library(filter_pointcloud_component SHARED src/filter_pointcloud.cpp)
target_compile_definitions(filter_pointcloud_component PRIVATE COMPILE_AS_COMPONENT)
target_link_libraries(filter_pointcloud_component TBB::tbb)
ament_target_dependencies(filter_pointcloud_component rclcpp tf2_ros sensor_msgs rclcpp_components visualization_msgs tf2_eigen spot_msgs)
rclcpp_components_register_nodes(filter_pointcloud_component "spot_navigation::PointcloudFilterComponent")

add_executable(filter_pointcloud_node src/filter_pointcloud.cpp)
target_compile_definitions(filter_pointcloud_node PRIVATE COMPILE_AS_NODE)
target_link_libraries(filter_pointcloud_node TBB::tbb)
ament_target_dependencies(filter_pointcloud_node rclcpp tf2_ros sensor_msgs rclcpp_components visualization_msgs tf2_eigen spot_msgs)

add_library(spot_controller SHARED src/spot_controller.cpp)
target_compile_definitions(spot_controller PUBLIC PLUGINLIB__DISABLE_BOOST_FUNCTIONS)
ament_target_dependencies(spot_controller rclcpp nav2_core nav2_msgs nav_msgs spot_msgs pluginlib tf2_ros)

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
  TARGETS 
    filter_pointcloud_component
    spot_controller
  DESTINATION lib
)

install(
  TARGETS filter_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_libraries(spot_controller)
ament_export_dependencies(rclcpp nav2_core nav2_msgs nav_msgs spot_msgs pluginlib tf2_ros)

pluginlib_export_plugin_description_file(nav2_core controller_plugin_registration/spot_controller.xml)

ament_package()
