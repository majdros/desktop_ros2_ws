import os
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    
    urdf_path = os.path.join(get_package_share_path('my_robot_description'),
                            'urdf', 'my_robot.urdf.xacro')
    
    rviz_config_path = os.path.join(get_package_share_path('my_robot_description'),
                                    'config', 'my_robot.rviz')
    
    rplidar_c1_launch_path = os.path.join(get_package_share_path('my_robot_description'),
                                        'launch', 'rplidar.launch.py')
    
    joystick_launch_path = os.path.join(get_package_share_path('my_robot_description'),
                                                'launch', 'joystick.launch.py')
    
    bno055_launch_path = os.path.join(get_package_share_path('bno055'),
                                        'launch', 'bno055.launch.py')


    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description,
                    'use_sim_time': False, 'queue_size': 200}]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory('my_robot_description'),'config','my_controllers.yaml') 

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    controller_params_file],
        output='screen'
    )

    delayed_controller_manager = TimerAction(period=2.5, actions=[controller_manager])

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        rviz2_node,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_c1_launch_path)
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joystick_launch_path)
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bno055_launch_path)
        )
    ])