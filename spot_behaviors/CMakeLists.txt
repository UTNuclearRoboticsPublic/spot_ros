cmake_minimum_required(VERSION 3.8)
project(spot_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 20)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spot_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen_stl_containers REQUIRED)
find_package(spot_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/check_battery.cpp
  src/dock_robot.cpp
  src/get_spot_ik.cpp
  src/walk_to_pose.cpp
  src/check_arm_stowed.cpp
  src/move_hand_to_pose.cpp
  src/check_hand_collision.cpp
  src/move_hand_through_poses.cpp
  src/gesture_sequence.cpp
  src/toggle_payload.cpp
  src/get_images.cpp
  src/get_gripper_holding_state.cpp
  src/get_stow_state.cpp
  src/set_simulated_pose.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2_kdl
  tf2_eigen
  spot_msgs
  moveit_msgs
  behaviortree_cpp
  tf2_geometry_msgs
  moveit_kinematics
  eigen_stl_containers
  spot_utils
  sensor_msgs
  std_msgs
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(spot_msgs spot_utils behaviortree_cpp tf2_kdl moveit_msgs moveit_kinematics sensor_msgs std_msgs)

# Install include files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
