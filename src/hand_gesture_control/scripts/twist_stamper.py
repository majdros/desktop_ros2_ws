#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header


class TwistStamperNode(Node):
    def __init__(self):
        super().__init__('twist_stamper_node')

        self.stamped_twist_pub = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)
        self.twist_sub = self.create_subscription(Twist, '/cmd_vel_gesture', self.twist_callback, 10)
        self.twist_sub      # avoid unused variable warning

        self.declare_parameter('twist_stamper_frame_id', '')
        self.twist_stamper_frame_id = str(self.get_parameter('twist_stamper_frame_id').value)
        self.get_logger().info(f'twist_stamper_frame_id: {self.twist_stamper_frame_id}')

    def twist_callback(self, twist_msg):
        try:
            output_msg = TwistStamped()
            output_msg.header = Header()
            output_msg.header.stamp = self.get_clock().now().to_msg()
            output_msg.header.frame_id = self.twist_stamper_frame_id
            output_msg.twist = twist_msg
            self.stamped_twist_pub.publish(output_msg)
        except Exception as e:
            self.get_logger().error(f'fError in image_callback: {str(e)}')



def main(args=None):
    rclpy.init(args=args)

    twist_stamper = TwistStamperNode()
    rclpy.spin(twist_stamper)
    twist_stamper.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()