cmake_minimum_required(VERSION 3.8)
project(hand_gesture_control)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/HandLandmark.msg"
    "msg/HandLandmarks.msg"
)

install(DIRECTORY
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/hand_tracking_node.py
    scripts/gesture_control_node.py
    scripts/twist_stamper.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch msg resource test
    DESTINATION share/${PROJECT_NAME}
)

ament_package()