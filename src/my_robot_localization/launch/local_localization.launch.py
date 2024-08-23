from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, unless_condition
import os

def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value= "True"
    )

    use_python = LaunchConfiguration("use_pyhton")

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transfrom_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0.103", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                    "--frame-id", "footprint_link_ekf", "--child-fram-id", "imu_link_ekf"]
    )

    robot_localizaiton = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("my_robot_localization"), "config", "ekf.yaml")]
    )

    imu_republisher_py = Node(
        package="my_robot_localization",
        executable="imu_republisher.py",
        condition=IfCondition(use_python)
    )

    return LaunchDescription([
        use_python_arg,
        static_transform_publisher,
        robot_localizaiton,
        imu_republisher_py
    ])