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

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/check_battery.cpp
  src/dock_robot.cpp
  src/trigger_service.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  spot_msgs
  behaviortree_cpp
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(spot_msgs behaviortree_cpp)

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

ament_package()
