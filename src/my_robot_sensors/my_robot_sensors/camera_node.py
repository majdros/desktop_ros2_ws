#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header
import cv2 as cv
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
import yaml
import os


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Params
        self.declare_parameter(name = 'camera_index', value = 0)
        self.declare_parameter(name = 'frame_rate', value = 30.0)
        self.declare_parameter(name = 'use_raw_image_publisher', value = True)
        self.declare_parameter(name = 'use_compressed_image_publisher', value = True)
        self.declare_parameter(name = 'web_camera_calibration_file', value = 'web_cam.yaml')
        self.declare_parameter(name = 'usb_camera_calibration_file', value = 'usb_cam.yaml')
        self.declare_parameter(name = 'jpeg_quality_value',
                            value = 70,       # (0 to 100)
                            descriptor = ParameterDescriptor(description = 'sets the value of the Quality from jpeg for the Encoding the Image as Compressed-Image'))

        self.jpeg_quality_value = self.get_parameter('jpeg_quality_value').get_parameter_value().integer_value
        self.use_raw_image_publisher = self.get_parameter('use_raw_image_publisher').get_parameter_value().bool_value
        self.use_compressed_image_publisher = self.get_parameter('use_compressed_image_publisher').get_parameter_value().bool_value
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        self.web_camera_calibration_file= self.get_parameter('web_camera_calibration_file').get_parameter_value().string_value
        self.usb_camera_calibration_file= self.get_parameter('usb_camera_calibration_file').get_parameter_value().string_value

        self.raw_image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.compressed_image_pub = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        self.camera_info_pub= self.create_publisher(CameraInfo, 'camera_info', 10)
        self.rectified_image_pub = self.create_publisher(Image, 'image_rect', 10)

        self.bridge = CvBridge()
        self.cap = cv.VideoCapture(self.camera_index, cv.CAP_V4L2)

        timer_period = 1 / self.frame_rate  # 30 FPS

        if self.use_raw_image_publisher or self.use_compressed_image_publisher:
            self.timer = self.create_timer(timer_period, self.publish_images)

        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
            rclpy.shutdown()
            exit(1)

        if self.cap.isOpened():
            self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv.CAP_PROP_FPS, self.frame_rate)
            self.cap.set(cv.CAP_PROP_BUFFERSIZE, 1)

            self.width = int(self.cap.get(cv.CAP_PROP_FRAME_WIDTH))
            self.height = int(self.cap.get(cv.CAP_PROP_FRAME_HEIGHT))

        if self.width != 640 or self.height != 480:
            self.get_logger().warn(f'Camera resolution mismatch!')
            self.get_logger().warn(f'Calibration is for 640x480, but camera delivers {self.width}x{self.height}')
        else:
            self.get_logger().info("ALLES SUPER!")

        # Load camera calibration
        self.camera_info_msg = self.load_camera_calibration()

        self.setup_undistortion_maps()


    def load_camera_calibration(self):
        try:
            # Get package share directory
            package_share_directory = get_package_share_directory('my_robot_sensors')
            calibration_file_path = os.path.join(package_share_directory, 'config', self.usb_camera_calibration_file)

            with open(calibration_file_path, 'r') as file:
                calib_data = yaml.safe_load(file)

            camera_info_msg = CameraInfo()

            camera_info_msg.width = calib_data['image_width']
            camera_info_msg.height = calib_data['image_height']
            camera_info_msg.distortion_model = calib_data['distortion_model']

            # Camera matrix (K)
            camera_info_msg.k = calib_data['camera_matrix']['data']

            # Distortion coefficients (D)
            camera_info_msg.d = calib_data['distortion_coefficients']['data']

            # Rectification matrix (R)
            camera_info_msg.r = calib_data['rectification_matrix']['data']

            # Projection matrix (P)
            camera_info_msg.p = calib_data['projection_matrix']['data']

            self.get_logger().info(f'Camera calibration loaded from {calibration_file_path}')
            return camera_info_msg

        except Exception as e:
            self.get_logger().error(f'Error in load_camera_calibration: {str(e)}')


    def setup_undistortion_maps(self):
        try:
            if hasattr(self, 'camera_info_msg'):
                self.camera_matrix = np.array(self.camera_info_msg.k).reshape(3, 3)
                self.dist_coeffs = np.array(self.camera_info_msg.d)
                self.rect_matrix = np.array(self.camera_info_msg.r).reshape(3, 3)

            # Precompute undistortion maps for efficiency
            self.map1, self.map2 = cv.initUndistortRectifyMap(
                self.camera_matrix, self.dist_coeffs, self.rect_matrix, self.camera_matrix, 
                (self.width, self.height), cv.CV_16SC2
            )
            self.get_logger().info(f'Undistortion maps created successfully for size {self.width}x{self.height}')

        except Exception as e:
            self.get_logger().error(f'Error in setup_undistortion_maps: {str(e)}')


    def publish_images(self):
        try:
            ret, frame = self.cap.read()
            if ret:
                timestamp = self.get_clock().now().to_msg()
                
                if self.use_raw_image_publisher:
                    self.raw_image_publisher(frame, timestamp)
                
                if self.use_compressed_image_publisher:
                    self.compressed_image_publisher(frame, timestamp)
                
                self.camera_info_publisher(timestamp)
                
                self.rectified_image_publisher(frame, timestamp)

        except Exception as e:
            self.get_logger().error(f'Error in publish_images: {str(e)}')


    def rectified_image_publisher(self, frame, timestamp):
        try:
            # Apply undistortion using precomputed maps
            rectified_frame = cv.remap(frame, self.map1, self.map2, cv.INTER_LINEAR)

            msg = self.bridge.cv2_to_imgmsg(rectified_frame, "bgr8")

            msg.header = Header()
            msg.header.stamp = timestamp
            msg.header.frame_id = "camera_optical"

            self.rectified_image_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error in rectified_image_publisher: {str(e)}')


    def raw_image_publisher(self, frame, timestamp):
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8") # for Debugging with rqt_image_view

            msg.header = Header()
            msg.header.stamp = timestamp
            msg.header.frame_id = "camera_optical"

            self.raw_image_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error in raw_image_publisher: {str(e)}')


    def compressed_image_publisher(self, frame, timestamp):
        try:
            encode_param = [cv.IMWRITE_JPEG_QUALITY, self.jpeg_quality_value]    
            # Encode the image as JPEG
            success, encoded_image = cv.imencode('.jpeg', frame, encode_param)
            if not success:
                self.get_logger().error('Failed to encode image as JPEG')
                return

            # Create a CompressedImage message
            header = Header()
            header.stamp = timestamp
            header.frame_id = "camera_optical"

            Compressed_img = CompressedImage()
            Compressed_img.header = header
            Compressed_img.format = "jpeg"

            # Converting the array to bytes.
            Compressed_img.data = encoded_image.tobytes()

            self.compressed_image_pub.publish(Compressed_img)

        except Exception as e:
            self.get_logger().error(f'Error in compressed_image_publisher: {str(e)}')


    def camera_info_publisher(self, timestamp):
        try:
            self.camera_info_msg.header.stamp= timestamp
            self.camera_info_msg.header.frame_id = 'camera_optical'

            self.camera_info_pub.publish(self.camera_info_msg)

        except Exception as e:
            self.get_logger().error(f'Error in camera_info_publisher: {str(e)}')


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