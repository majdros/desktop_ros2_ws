#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from hand_gesture_control.msg import HandLandmarks


class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control_node')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.landmark_sub = self.create_subscription(HandLandmarks, '/hand_landmarks', self.landmark_callback, 10)
        self.landmark_sub
        self.total_fingers = 0


    def landmark_callback(self, landmark_msg):
        try:
            landmarks = landmark_msg.landmarks
            gesture = self.recognize_gesture(landmarks)
            if gesture:
                cmd_vel = self.gesture_to_twist(gesture)
                self.cmd_vel_pub.publish(cmd_vel)
        except Exception as e:
            self.get_logger().error(f'Error in landmark_callback: {str(e)}')


    def recognize_gesture(self, landmarks):
        tipIds = [4, 8, 12, 16, 20]

        if landmarks:
            fingers = []
            landmark_dict = {lm.id: lm for lm in landmarks}

            # thumb- spezieller Fall wegen x-Koordinat
            if landmark_dict[tipIds[0]].x > (landmark_dict[tipIds[0]-1].x):
                    fingers.append(1)
            else:
                fingers.append(0)

            # rest 4 fingers
            for id in range(1, len(tipIds)):
                tip_y = landmark_dict[tipIds[id]].y
                pip_y = landmark_dict[tipIds[id]-2].y

                if tip_y < pip_y: 
                    fingers.append(1)
                else:
                    fingers.append(0)

            self.total_fingers = fingers.count(1)
            self.get_logger().info(str(self.total_fingers))

        # Map finger count to gestures
        return {
            0: 'forward',
            1: 'forward',
            2: 'backward',
            3: 'left',
            4: 'right',
            5: 'stop',
        }.get(self.total_fingers, 'stop')



    def gesture_to_twist(self, gesture):
        cmd_vel = Twist()
        if gesture == "forward":
            cmd_vel.linear.x = 1.1
            cmd_vel.angular.z = 0.0
        elif gesture == "backward":
            cmd_vel.linear.x = -1.1
            cmd_vel.angular.z = 0.0
        elif gesture == "left":
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 1.7
        elif gesture == "right":
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -1.7
        elif gesture == "stop":
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        return cmd_vel



def main(args=None):
    rclpy.init(args=args)

    node = GestureControlNode()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()