import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def generate_launch_description():

    teleop_twist_params = os.path.join(get_package_share_directory('my_robot_description'), 'config', 'teleop_twist_keyboard.yaml')

    teleop_twist_keyboard = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
            '--ros-args',
            '--params-file', teleop_twist_params,
            '-r', '/cmd_vel:=/cmd_vel_stamped'
        ],
        output='screen',
        shell=True
    )




    return LaunchDescription([
        teleop_twist_keyboard,
    ])