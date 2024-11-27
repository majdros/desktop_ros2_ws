from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():


    robot_localizaiton = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("my_robot_localization"), "config", "ekf.yaml")]
    )


    return LaunchDescription([
        robot_localizaiton,
    ])