#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
import HandTrackingModule as htm
from hand_gesture_control.msg import HandLandmark, HandLandmarks
import time


class HandTrackingNode(Node):
    """A ROS2 node for real-time hand tracking and landmark detection.

    This node subscribes to camera images from '/image_raw' topic and processes them
    using MediaPipe hand tracking. It detects hand landmarks and publishes them as
    custom ROS2 messages to '/hand_landmarks' topic.

    Attributes:
        bridge (CvBridge): Bridge for converting between ROS and OpenCV images
        hand_detector (HandDetector): MediaPipe-based hand detection and tracking module
        landmark_pub (Publisher): Publisher for hand landmark messages
        image_sub (Subscription): Subscriber for camera images
        pTime (float): Previous time for FPS calculation
        draw_enabled (bool): Flag to enable/disable visualization

    Publishers:
        /hand_landmarks (HandLandmarks): Publishes detected hand landmarks

    Subscribers:
        /image_raw (Image): Subscribes to camera image feed

    Custom Messages:
        HandLandmark: Single landmark with id, x, y coordinates
        HandLandmarks: Array of hand landmarks
    """

    def __init__(self):
        super().__init__('hand_tracking_node')
        self.bridge = CvBridge()

        self.hand_detector = htm.HandDetector(maxHands=1, detectionCon=0.9, trackingCon=0.9)

        self.landmark_pub = self.create_publisher(HandLandmarks, '/hand_landmarks', 10)
    
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.image_sub
        self.pTime = 0
        self.declare_parameter('draw_enabled', True)        # To show cv2.VideoCapture
        self.draw_enabled = self.get_parameter('draw_enabled')._value
        self.get_logger().info(f'Visualization enabled: {self.draw_enabled}')

    def image_callback(self, img):
        try:
            # Convert ROS Image to OpenCV format
            frame_bgr = self.bridge.imgmsg_to_cv2(img, "bgr8")

            frame_processed = self.hand_detector.findHands(frame_bgr,draw=self.draw_enabled)
            hands_landmarks = self.hand_detector.findPosition(frame_processed)

            # Convert Python liste hands_landmarks to ROS2-Message
            if hands_landmarks:
                landmarks_msg = HandLandmarks()

                for landmark_point in hands_landmarks[0]:      # Only first hand
                        landmark = HandLandmark()
                        landmark.id = landmark_point[0]
                        landmark.x = landmark_point[1]
                        landmark.y = landmark_point[2]
                        landmarks_msg.landmarks.append(landmark)

                self.landmark_pub.publish(landmarks_msg)
                self.get_logger().debug(f'Published {len(landmarks_msg.landmarks)} landmarks')

            if self.draw_enabled:
                cTime = time.time()
                fps = 1/(cTime - self.pTime)
                self.pTime = cTime

                cv.putText(frame_processed, f'FPS: {str(int(fps))}',
                        (50, 30), cv.FONT_HERSHEY_COMPLEX, 0.5, (255,0,255), 1)

                cv.imshow("Hand Tracking", frame_processed)
                if cv.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info('Shutting down')
                    self.destroy_node()
                    exit()

        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {str(e)}')



def main(args=None):
    rclpy.init(args=args)

    node = HandTrackingNode()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()