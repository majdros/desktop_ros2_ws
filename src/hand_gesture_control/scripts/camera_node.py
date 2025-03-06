#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header
import cv2 as cv


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv.VideoCapture(0, cv.CAP_V4L2)

        timer_period = 0.066  # 30 FPS
        self.timer = self.create_timer(timer_period, self.publish_raw_image)

        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
            exit()


    def publish_raw_image(self):
        try:
            ret, frame = self.cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8") # for Debugging with rqt_image_view

                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_frame"
                self.image_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error in publish_raw_image: {str(e)}')


    def __del__(self):
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)

    raw_image_publisher = ImagePublisher()
    rclpy.spin(raw_image_publisher)
    raw_image_publisher.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()