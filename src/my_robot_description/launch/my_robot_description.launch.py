from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    urdf_path = os.path.join(get_package_share_path('my_robot_description'),
                            'urdf', 'my_robot.urdf.xacro')
    
    rviz_config_path = os.path.join(get_package_share_path('my_robot_description'),
                                    'config', 'my_robot.rviz')
    
    joystick_launch_path = os.path.join(get_package_share_path('my_robot_description'),
                                                'launch', 'joystick.launch.py')
    
    bno055_launch_path = os.path.join(get_package_share_path('bno055'),
                                        'launch', 'bno055.launch.py')

    robot_name_in_model = 'my_robot'
    sdf_model_path = os.path.join(get_package_share_directory('my_robot_description'),
                                'models', 'model.sdf')



    # Declare the launch argument
    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control if true'
    )
    
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description,
                    'use_sim_time': False, 'queue_size': 200, 'publish_frequency': 30.0}],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    gazebo_params_file = os.path.join(
                    get_package_share_directory('my_robot_description'), 'config', 'gazebo_params.yaml')

    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
        launch_arguments={'extra_gazebo_args': '-s libgazebo_ros_init.so -s libgazebo_ros_factory.so --ros-args --params-file ' + gazebo_params_file}.items()
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'))
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-file', sdf_model_path,
            '-x', '0.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0'
        ],
        output='screen'
    )

    twist_stamper= Node(
    package='twist_stamper',
    executable='twist_unstamper',
    remappings=[('/cmd_vel_in','/cmd_vel_stamped'),
                ('/cmd_vel_out','/cmd_vel_unstamped')]
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
        robot_state_publisher_node,
        # use_ros2_control_arg,

        rviz2_node,
        twist_stamper,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bno055_launch_path)
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(joystick_launch_path)
        # )

        # start_gazebo_server_cmd,
        # start_gazebo_client_cmd,
        # spawn_entity_cmd,

        # static_transform_publisher_base_footprint_to_laser,
        # static_transform_publisher_base_footprint_to_base_link,
    ])