#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Header
import cv2 as cv
from rcl_interfaces.msg import ParameterDescriptor



class ImagePublisher(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Params
        self.declare_parameter(name = 'camera_index', value = 0)
        self.declare_parameter(name = 'frame_rate', value = 30.0)
        self.declare_parameter(name = 'use_raw_image_publisher', value = False)
        self.declare_parameter(name = 'use_compressed_image_publisher', value = True)
        self.declare_parameter(name = 'jpeg_quality_value',
                            value = 70,       # (0 to 100)
                            descriptor = ParameterDescriptor(description = 'sets the value of the Quality from jpeg for the Encoding the Image as Compressed-Image'))

        self.jpeg_quality_value = self.get_parameter('jpeg_quality_value').get_parameter_value().integer_value
        self.use_raw_image_publisher = self.get_parameter('use_raw_image_publisher').get_parameter_value().bool_value
        self.use_compressed_image_publisher = self.get_parameter('use_compressed_image_publisher').get_parameter_value().bool_value
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value


        self.raw_image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.compressed_image_pub = self.create_publisher(CompressedImage, '/image_raw/compressed', 10)

        self.bridge = CvBridge()
        self.cap = cv.VideoCapture(self.camera_index, cv.CAP_V4L2)

        timer_period = 1 / self.frame_rate  # 30 FPS

        if self.use_raw_image_publisher:
            self.timer = self.create_timer(timer_period, self.raw_image_publisher)
        if self.use_compressed_image_publisher:
            self.timer = self.create_timer(timer_period, self.compressed_image_publisher)

        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
            exit()


    def raw_image_publisher(self):
        try:
            ret, frame = self.cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8") # for Debugging with rqt_image_view

                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_frame"
                self.raw_image_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error in raw_image_publisher: {str(e)}')


    def compressed_image_publisher(self):
        try:
            ret, frame = self.cap.read()
            if ret:
                encode_param = [cv.IMWRITE_JPEG_QUALITY, self.jpeg_quality_value]    
                # Encode the image as JPEG
                success, encoded_image = cv.imencode('.jpeg', frame, encode_param)
                if not success:
                    self.get_logger().error('Failed to encode image as JPEG')
                    return

                # Create a CompressedImage message
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = "camera_frame"

                Compressed_img = CompressedImage()
                Compressed_img.header = header
                Compressed_img.format = "jpeg"

                # Converting the array to bytes.
                Compressed_img.data = encoded_image.tobytes()

                self.compressed_image_pub.publish(Compressed_img)

        except Exception as e:
            self.get_logger().error(f'Error in compressed_image_publisher: {str(e)}')


    def __del__(self):
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()