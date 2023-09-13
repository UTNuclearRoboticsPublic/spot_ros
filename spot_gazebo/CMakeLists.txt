cmake_minimum_required(VERSION 3.5)
project(spot_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds rviz models
        DESTINATION share/${PROJECT_NAME})

ament_package()
