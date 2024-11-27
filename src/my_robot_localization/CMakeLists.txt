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

include_directories(include)

ament_python_install_package(${PROJECT_NAME})

# Installation der Python-Skripte
install(PROGRAMS
  ${PROJECT_NAME}/kalman_filter_real.py
  ${PROJECT_NAME}/kalman_filter_sim.py
  ${PROJECT_NAME}/noisy_odometry_publisher.py
  ${PROJECT_NAME}/imu_republisher.py
  ${PROJECT_NAME}/odometry_motion_model.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installation des Include-Ordners
install(
  DIRECTORY include
  DESTINATION include
)

# Installation der Launch- und Config-Dateien
install(DIRECTORY
  launch 
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
