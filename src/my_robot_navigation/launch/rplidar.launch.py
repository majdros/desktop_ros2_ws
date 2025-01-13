import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rplidar_node = Node(
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
                'inverted': False,          # Inverted: True if the LIDAR is mounted upside down
                'flip_x_axis': True,         # Flip X axis: True if the LIDAR Data is flipped in X axis
                # 'scan_frequency': 5.0
                'use_sim_time': False,
            }]
    )


    return LaunchDescription([
        rplidar_node,
    ])