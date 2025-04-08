import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    bno055_config = os.path.join(get_package_share_directory('my_robot_sensors'),
            'config', 'bno055_params.yaml')


    bno055_node = Node(
        package = 'my_robot_sensors',
        executable = 'bno055',
        parameters = [bno055_config]
    )


    return LaunchDescription([
        bno055_node,
    ])