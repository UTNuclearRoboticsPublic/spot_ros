cmake_minimum_required(VERSION 3.8)
project(spot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find ROS dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Non-ROS dependencies
find_package(OpenCV REQUIRED)
# Specify include directories
include_directories(
  include
)

# Enable position-independent code (PIC) generation for all targets (needed for shared libraries).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add libraries
add_library(camera_client src/${PROJECT_NAME}/camera_client.cpp)

# Link against external libraries
target_link_libraries(camera_client ${OpenCV_LIBRARIES})

# Link against ROS dependencies
ament_target_dependencies(camera_client rclcpp sensor_msgs cv_bridge)

# Install the 'include' directory
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include)
install(DIRECTORY include/base64
        DESTINATION include)

# Install the library and its headers
install(TARGETS camera_client
  EXPORT ${PROJECT_NAME} # Export for visibility with other CMake packages
  ARCHIVE DESTINATION lib  # Static library installation path
  LIBRARY DESTINATION lib  # Dynamic library installation path
  RUNTIME DESTINATION bin  # Executable installation path
  INCLUDES DESTINATION include # Header files
)

# Export dependencies for downstream packages
ament_export_dependencies(sensor_msgs cv_bridge)

# Export the group for downstream packages
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
