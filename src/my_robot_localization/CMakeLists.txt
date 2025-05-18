cmake_minimum_required(VERSION 3.8)
project(my_robot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_map_server REQUIRED)

include_directories(include)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/kalman_filter.py
  ${PROJECT_NAME}/velocity_comparison_node.py
  ${PROJECT_NAME}/innovation_evaluator.py
  
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(DIRECTORY
  launch 
  config
  bag_files
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
