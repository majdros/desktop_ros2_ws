#!/bin/bash

set -e  # Exit on error

ROS_DISTRO=${ROS_DISTRO:-humble}

# Installing base ROS2 and build tools
apt-get update && apt-get install -y \
    python3-serial \
    build-essential \
    bash-completion \
    python3-colcon-common-extensions

# Installing Generaging packages
apt-get install -y \
    ros-${ROS_DISTRO}-builtin-interfaces \
    ros-${ROS_DISTRO}-rosidl-generator-c \
    ros-${ROS_DISTRO}-rosidl-default-generator \
    ros-${ROS_DISTRO}-rosidl-default-runtime

# Installing Navigation packages
apt-get install -y \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager

apt-get install -y \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-twist-stamper

apt-get install -y \
    ros-${ROS_DISTRO}-rplidar-ros \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge

# Clean up
rm -rf /var/lib/apt/lists/*