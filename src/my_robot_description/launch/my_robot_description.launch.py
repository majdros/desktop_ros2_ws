import os
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition 


def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('my_robot_description'),
                            'urdf', 'my_robot.urdf.xacro')
    
    rviz_config_path = os.path.join(get_package_share_path('my_robot_description'),
                                    'config', 'my_robot_description.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )


    robot_name_in_model = 'my_robot'

    sdf_model_path = os.path.join(get_package_share_directory('my_robot_description'),
                                'models', 'my_robot_model.sdf')

    use_gazebo = DeclareLaunchArgument(
        'use_gazebo',
        default_value= 'false',
        description= 'Enables Gazebo if true'
    )

    use_gazebo_config = LaunchConfiguration('use_gazebo')

    gazebo_params_file = os.path.join(
                    get_package_share_directory('my_robot_description'), 'config', 'gazebo_params.yaml')

    gazebo_world_path = os.path.join(
                    get_package_share_directory('my_robot_description'), 'config', 'gazebo.world')

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description,
                    'use_sim_time': use_gazebo_config, 'queue_size': 200, 'publish_frequency': 30.0}],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': gazebo_world_path,
            'extra_gazebo_args': '-s libgazebo_ros_init.so -s libgazebo_ros_factory.so --ros-args --params-file ' + gazebo_params_file
            }.items(),
        condition=IfCondition(use_gazebo_config)
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')),
        condition=IfCondition(use_gazebo_config)
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-file', sdf_model_path,
            '-x', '0.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0'
        ],
        output='screen',
        condition=IfCondition(use_gazebo_config)
    )



    static_transform_publisher_base_footprint_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.1955", "0.0", "0.0", "0.0", "base_footprint", "laser"]
    )


    static_transform_publisher_base_footprint_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0125", "0.0", "0.0", "0.0", "base_footprint", "base_link"]
    )



    return LaunchDescription([
        use_gazebo,
        robot_state_publisher_node,
        rviz2_node,

        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity_cmd,

        # static_transform_publisher_base_footprint_to_laser,
        # static_transform_publisher_base_footprint_to_base_link,
    ])