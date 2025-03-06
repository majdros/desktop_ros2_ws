from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camer_node = Node(
        package = 'hand_gesture_control',
        executable = 'camera_node.py' 
    )

    hand_tracking_node = Node(
        package = 'hand_gesture_control',
        executable = 'hand_tracking_node.py' 
    ) 

    gesture_control_node = Node(
        package = 'hand_gesture_control',
        executable = 'gesture_control_node.py' 
    ) 

    twist_stamper = Node(
        package = 'hand_gesture_control',
        executable = 'twist_stamper.py' 
    )


    return LaunchDescription([
        camer_node,
        hand_tracking_node,
        gesture_control_node,
        twist_stamper
    ])