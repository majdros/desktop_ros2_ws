from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    odom_laser_broadcaster = Node(
            package='transform_broadcaster',
            executable='odom_laser_broadcaster',
            output='screen',
            name = 'odom_laser_broadcaster',
    )


    return LaunchDescription([
        odom_laser_broadcaster,
    ])