import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    teleop_twist_params = os.path.join(
        get_package_share_directory('my_robot_control'),
        'config', 'teleop_twist_keyboard.yaml'
    )

    teleop_twist_keyboard = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--disable-factory',
            '--',
            'bash', '-c',
            'ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args '
            # '--params-file ' + teleop_twist_params +
            ' --remap /cmd_vel:=/cmd_vel_key; exec bash'
        ],
        output='screen'
    )


    return LaunchDescription([
        teleop_twist_keyboard
        ])