import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rplidar_config = os.path.join(get_package_share_directory('my_robot_sensors'),
        'config', 'rplidar_params.yaml')


    rplidar_node = Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters= [rplidar_config]
    )


    return LaunchDescription([
        rplidar_node,
    ])