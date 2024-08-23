from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription 
from launch.conditions import IfCondition
import xacro
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    urdf_path = os.path.join(get_package_share_path('my_robot_description'),
                            'urdf', 'my_robot.urdf.xacro')
    
    rviz_config_path = os.path.join(get_package_share_path('my_robot_description'),
                                    'config', 'my_robot.rviz')
    
    view_rplidar_c1_launch_path = os.path.join(get_package_share_path('rplidar_ros'),
                                                'launch', 'view_rplidar_c1_launch.py')
    
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
                    'use_sim_time': True, 'queue_size': 200}]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    gazebo_params_file = os.path.join(
                    get_package_share_directory('my_robot_description'), 'config', 'gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file' + gazebo_params_file}.items()
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "robot_description", "-entity", "my_robot_description"]
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_broad"],
    )


    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        use_ros2_control_arg,
        rviz2_node,
        spawn_entity_node,
        diff_drive_spawner,
        joint_broad_spawner,
        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        #     output='screen'
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(view_rplidar_c1_launch_path)
        )
    ])