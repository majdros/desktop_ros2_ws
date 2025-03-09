from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Available Launch Parameters
    use_twist_stamper = DeclareLaunchArgument(
        'use_twist_stamper', 
        default_value= 'true', 
        description= 'Disables twist_stamper if false')

    twist_stamper_frame_id = DeclareLaunchArgument(
        'twist_stamper_frame_id', 
        default_value= '', 
        description= 'Frame ID for TwistStamped messages')

    draw_enabled = DeclareLaunchArgument(
        'draw_enabled', 
        default_value= 'true', 
        description= 'Enable visualization of hand tracking if true')


    # Available Nodes
    camer_node = Node(
        package = 'hand_gesture_control',
        executable = 'camera_node.py' 
    )

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
        camer_node,
        hand_tracking_node,
        gesture_control_node,
        twist_stamper
    ])