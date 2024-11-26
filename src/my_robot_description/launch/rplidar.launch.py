import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'channel_type': 'serial',
                'serial_baudrate': 460800,
                'inverted': False
            }]
        )
    ])
