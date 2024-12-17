from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "-0.05", "--y", "0","--z", "0.152", # Values from Urdf between imu-link (bno055) und #footprint-link#
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                    "--frame-id", "base_footprint_ekf", ######
                    "--child-frame-id", "bno055_ekf"],
    )

    robot_localizaiton = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("my_robot_localization"), "config", "ekf.yaml")],
        # remappings=[("odometry/filtered", "odometry/local")],
    )

    imu_republisher = Node(
        package="my_robot_localization",
        executable="imu_republisher.py",
        name="imu_republisher",
    )


    return LaunchDescription([
        # static_transform_publisher,
        robot_localizaiton,
        # imu_republisher
    ])