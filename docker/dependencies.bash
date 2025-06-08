#!/bin/bash

set -e
ROS_DISTRO=${ROS_DISTRO:-humble}

# Update package lists
apt-get update

# Installing base ROS2 and build tools
    apt-get install -y \
    ros-$ROS_DISTRO-rosidl-default-generators \
    ros-$ROS_DISTRO-rosidl-default-runtime \
    ros-$ROS_DISTRO-rosidl-typesupport-c \
    ros-$ROS_DISTRO-rosidl-typesupport-fastrtps-c \
    ros-$ROS_DISTRO-rosidl-typesupport-fastrtps-cpp \
    ros-$ROS_DISTRO-rosidl-typesupport-introspection-c \
    ros-$ROS_DISTRO-rosidl-typesupport-introspection-cpp \
    ros-$ROS_DISTRO-rclcpp \
    python3-serial \
    python3-smbus \
    bash-completion \
    libboost-system-dev \
    python3-argcomplete \
    ros-${ROS_DISTRO}-rqt-graph \
    ros-${ROS_DISTRO}-rqt-common-plugins 

# Installing Gazebo, rviz2 and OpenCV packages
apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-rviz2 \
    python3-opencv \
    libopencv-dev

# Installing Navigation packages
apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros

apt-get install -y \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-twist-stamper \
    ros-${ROS_DISTRO}-rosbridge-server 

apt-get install -y \
    ros-${ROS_DISTRO}-rplidar-ros \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-example-interfaces

apt-get install -y \
    python3-mediapipe \
    evtest \
    jstest-gtk \
    htop \
    tree \
    vim 

# Clean up
rm -rf /var/lib/apt/lists/*