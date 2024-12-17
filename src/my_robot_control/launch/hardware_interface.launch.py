from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('my_robot_description'),'urdf', 'my_robot.urdf.xacro')

    my_controllers_yaml_path = os.path.join(get_package_share_path('my_robot_description'), 'config', 'my_controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                    'use_sim_time': False, 'queue_size': 200}]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description' : robot_description,
            'use_sim_time':False},
            my_controllers_yaml_path,
        ]
    )

    hardware_interface_node = Node(
        package='my_robot_control',
        executable='hardware_interface',
        name='hardware_interface',
        output='screen',
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
        # hardware_interface_node,
        # joint_state_publisher_node
    ])