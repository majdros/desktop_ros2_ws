cmake_minimum_required(VERSION 3.8)
project(my_robot_slam_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

install(DIRECTORY
  config
  launch
  images
  DESTINATION share/${PROJECT_NAME}/
  )

  install(
    DIRECTORY maps
    DESTINATION share/${PROJECT_NAME}/
    FILES_MATCHING PATTERN "*.pgm" PATTERN "*.yaml"
  )

ament_package()
