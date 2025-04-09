from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os


def generate_launch_description():

    ekf_config = os.path.join(get_package_share_directory("my_robot_localization"), 
                        "config", "ekf.yaml")

    bno055_launch_path = os.path.join(get_package_share_directory('my_robot_sensors'),
                        'launch', 'bno055.launch.py')

    bno055_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(bno055_launch_path))

    robot_localizaiton = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[ekf_config],
    )
        # remappings=[("odometry/filtered", "odometry/local")],


    return LaunchDescription([
        bno055_node,
        robot_localizaiton,

    ])