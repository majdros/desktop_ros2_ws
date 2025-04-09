import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition 
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Available Launch Arguments
    use_teleop_keyboard = DeclareLaunchArgument(name ='use_teleop_keyboard', default_value= 'true')
    use_teleop_joy = DeclareLaunchArgument(name = 'use_teleop_joy', default_value= 'false')
    use_twist_stamper = DeclareLaunchArgument(name = 'use_twist_stamper', default_value= 'true')    
    twist_stamper_frame_id = DeclareLaunchArgument(name = 'twist_stamper_frame_id', default_value= 'base_link')

    # Launch Configurations
    use_teleop_keyboard_config = LaunchConfiguration('use_teleop_keyboard')
    use_teleop_joy_config = LaunchConfiguration('use_teleop_joy')
    use_twist_stamper_config = LaunchConfiguration('use_twist_stamper')
    twist_stamper_frame_id_config = LaunchConfiguration('twist_stamper_frame_id')

    # Params files
    joy_params = os.path.join(get_package_share_directory('my_robot_control'), 
                            'config', 'joystick.yaml')
    twist_mux_params = os.path.join(get_package_share_directory("my_robot_control"),
                                    "config", "twist_mux.yaml")


    # Nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
        condition = IfCondition(use_teleop_joy_config),
    )

    joy_teleop_node= Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[joy_params],
        condition = IfCondition(use_teleop_joy_config),
        remappings=[('/cmd_vel', '/cmd_vel_joy')]
    )


    key_teleop_launch_path = os.path.join(get_package_share_directory('my_robot_control'), 'launch', 'teleop_twist_keyboard.launch.py')
    key_teleop_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(key_teleop_launch_path),
        condition=IfCondition(use_teleop_keyboard_config)
    )


    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': False}]
    )

    twist_stamper = Node(
        package = 'twist_stamper',
        executable = 'twist_stamper',
        remappings= [('/cmd_vel_in', '/cmd_vel_out'),
                    ('/cmd_vel_out', '/cmd_vel_stamped')],
        condition = IfCondition(use_twist_stamper_config),
        parameters=[{
            'twist_stamper_frame_id': twist_stamper_frame_id_config
        }]
    )


    safety_stop_node = Node(
        package='my_robot_control',
        executable='safety_stop_node',
        name='safety_stop_node',
        parameters=[{'use_teleop_joy': use_teleop_joy_config,
                    'use_teleop_keyboard': use_teleop_keyboard_config}]
    )


    return LaunchDescription([
        use_teleop_keyboard,
        use_teleop_joy,
        use_twist_stamper,
        twist_stamper_frame_id,

        joy_node,
        joy_teleop_node,
        key_teleop_node,
        twist_mux,
        twist_stamper,
        safety_stop_node,


    ])