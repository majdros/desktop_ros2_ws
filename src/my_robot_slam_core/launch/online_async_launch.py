import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    slam_params_file_path = os.path.join(get_package_share_directory('my_robot_slam_core'),
                        'config', 'mapping_params_online_async.yaml')

    rplidar_launch_path = os.path.join(get_package_share_directory('my_robot_sensors'),
                        'launch','rplidar.launch.py')

    lifecycle_nodes = ['map_saver_server', 'slam_toolbox']


    # Declare Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock if true')

    slam_config_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value= slam_params_file_path,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    use_map_saver_arg = DeclareLaunchArgument(
        'use_map_saver',
        default_value = 'false',
        description = 'Enables map_saver_server to save a map if true')


    # Launch Configurations
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    slam_config = LaunchConfiguration('slam_params_file')
    use_map_saver_config = LaunchConfiguration('use_map_saver')


    # Available Nodes
    async_slam_toolbox_node = Node(
        package = 'slam_toolbox',
        executable = 'async_slam_toolbox_node',
        name = 'slam_toolbox',
        output = 'screen',
        parameters = [
            slam_config,
            {'use_sim_time': use_sim_time_config}
        ],
    )

    rplidar_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(rplidar_launch_path))

    nav2_map_saver = Node(
        package = 'nav2_map_server',
        executable = 'map_saver_server',
        name = 'map_saver_server',
        output = 'screen',
        condition = IfCondition(use_map_saver_config),
        parameters = [
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time_config},
            {"free_thresh_defaul": 0.196},
            {"occupied_thresh_default": 0.65},
            {"free_thresh": 0.25},
        ]
    )


    nav2_lifecycle_manager = Node(
        package=  'nav2_lifecycle_manager',
        executable = 'lifecycle_manager',
        name = 'lifecycle_manager',
        output = 'screen',
        parameters = [
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time_config},
        ]
    )


    return LaunchDescription([
    use_sim_time_arg,
    slam_config_arg,
    use_map_saver_arg,

    async_slam_toolbox_node,
    rplidar_node,
    nav2_map_saver,
    nav2_lifecycle_manager,
    ])