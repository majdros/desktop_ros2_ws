from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory


def generate_launch_description():

    rplidar_c1_launch_path = os.path.join(get_package_share_path('my_robot_navigation'),
                                        'launch', 'rplidar.launch.py')

    slam_toolbox_params_file = os.path.join(
        get_package_share_directory("my_robot_navigation"), "config", "mapper_params_online_async.yaml")

    slam_toolbox = Node(
        package = 'slam_toolbox',
        executable = 'async_slam_toolbox_node',
        name = 'slam_toolbox',
        parameters=[{'use_sim_time': False}, {'yaml_filename':slam_toolbox_params_file}],
    )









    return LaunchDescription([
        slam_toolbox,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_c1_launch_path)
        ),

    ])
