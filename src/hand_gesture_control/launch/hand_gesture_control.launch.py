import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    # Available Launch Parameters
    use_twist_stamper = DeclareLaunchArgument(
        'use_twist_stamper', 
        default_value= 'false', 
        description= 'Disables twist_stamper if false')

    twist_stamper_frame_id = DeclareLaunchArgument(
        'twist_stamper_frame_id', 
        default_value= '', 
        description= 'Frame ID for TwistStamped messages')

    draw_enabled = DeclareLaunchArgument(
        'draw_enabled', 
        default_value= 'true', 
        description= 'Enable visualization with cv.imshow of hand tracking if true')


    camera_launch_path= os.path.join(get_package_share_directory('my_robot_sensors'), 'launch', 'camera.launch.py')

    # Available Nodes
    camera_node= IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch_path))

    hand_tracking_node = Node(
        package = 'hand_gesture_control',
        executable = 'hand_tracking_node.py',
        parameters=[{
            'draw_enabled': LaunchConfiguration('draw_enabled')
        }]
    ) 

    gesture_control_node = Node(
        package = 'hand_gesture_control',
        executable = 'gesture_control_node.py' 
    ) 

    twist_stamper = Node(
        package = 'hand_gesture_control',
        executable = 'twist_stamper.py',
        condition = IfCondition(LaunchConfiguration('use_twist_stamper')),
        parameters=[{
            'twist_stamper_frame_id': LaunchConfiguration('twist_stamper_frame_id')
        }]
    )


    return LaunchDescription([
        use_twist_stamper,
        twist_stamper_frame_id,
        draw_enabled,
        camera_node,
        hand_tracking_node,
        gesture_control_node,
        twist_stamper
    ])