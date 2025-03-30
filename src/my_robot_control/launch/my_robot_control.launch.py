import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition 
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Available Launch Parameters
    use_teleop_keyboard = DeclareLaunchArgument(
        'use_teleop_keyboard',
        default_value= 'true',
        description= 'Disables teleop_twist_keyboard if false'
    )
    use_teleop_keyboard_config = LaunchConfiguration('use_teleop_keyboard')

    use_teleop_joy = DeclareLaunchArgument(
        'use_teleop_joy',
        default_value= 'false',
        description= 'Enable teleop_twist_joy if true'
    )
    use_teleop_joy_config = LaunchConfiguration('use_teleop_joy')

    use_twist_stamper = DeclareLaunchArgument(
        'use_twist_stamper', 
        default_value= 'true', 
        description= 'Disables twist_stamper if false')    

    twist_stamper_frame_id = DeclareLaunchArgument(
        'twist_stamper_frame_id', 
        default_value= 'base_link',
        description= 'Frame ID for TwistStamped messages')



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

    teleop_node= Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[joy_params],
        condition = IfCondition(use_teleop_joy_config),
        remappings=[('/cmd_vel', '/cmd_vel_joy')]
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
        condition = IfCondition(LaunchConfiguration('use_twist_stamper')),
        parameters=[{
            'twist_stamper_frame_id': LaunchConfiguration('twist_stamper_frame_id')
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
        teleop_node,
        twist_mux,
        twist_stamper,
        safety_stop_node,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('my_robot_control'), 'launch', 'teleop_twist_keyboard.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_teleop_keyboard'))
        ),
    ])