import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Declare Launch Arguemetns
    map_name_arg = DeclareLaunchArgument(
        name = 'map_name',
        default_value = 'office')

    use_sim_time_arg = DeclareLaunchArgument(
        name = 'use_sim_time',
        default_value = 'false',
        description = 'Use simulation/Gazebo clock if true')


    # Launch Configurations
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    map_name_config = LaunchConfiguration('map_name')

    # Available varibales
    map_path = PathJoinSubstitution([get_package_share_directory('my_robot_slam_core'),
                            'maps', map_name_config, 'map.yaml'])

    rplidar_launch_path = os.path.join(get_package_share_directory('my_robot_sensors'),
                        'launch','rplidar.launch.py')

    lifecycle_nodes = ['map_server', 'amcl']

    amcl_path = os.path.join(get_package_share_directory('my_robot_localization'),
                        'config', 'amcl.yaml')


    # Available Nodes
    rplidar_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(rplidar_launch_path))

    nav2_map_server = Node(
        package = 'nav2_map_server',
        executable = 'map_server',
        name = 'map_server',
        output = 'screen',
        parameters = [
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time_config},
        ]
    )

    nav2_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        emulate_tty=True,
        parameters=[
            amcl_path,
            {'use_sim_time': use_sim_time_config},
        ],
    )

    nav2_lifecycle_manager = Node(
        package=  'nav2_lifecycle_manager',
        executable = 'lifecycle_manager',
        name = 'lifecycle_manager',
        output = 'screen',
        parameters = [
            {'autostart': True},
            {'node_names': lifecycle_nodes},
            {'use_sim_time': use_sim_time_config},
        ]
    )


    return LaunchDescription([
    use_sim_time_arg,
    map_name_arg,

    rplidar_node,
    nav2_map_server,
    nav2_amcl,
    nav2_lifecycle_manager,
    ])