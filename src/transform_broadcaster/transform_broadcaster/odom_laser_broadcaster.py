#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from nav_msgs.msg import Odometry



class OdomToLaserBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_laser_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_subscriber = self.create_subscription(
            Odometry,'/odometry/filtered', self.handle_odom_msg, 50)
        
        self.get_logger().info('Odom to Laser Broadcaster has been started.')

        self.declare_parameter('laser_offset_x', 0.0)
        self.declare_parameter('laser_offset_y', 0.0)
        self.declare_parameter('laser_offset_z', 0.1955)


    def handle_odom_msg(self, msg):
        t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'laser'

        odom_msg = msg.pose.pose
        t.transform.translation.x = odom_msg.position.x + self.get_parameter('laser_offset_x').value
        t.transform.translation.y = odom_msg.position.y + self.get_parameter('laser_offset_y').value
        t.transform.translation.z =  self.get_parameter('laser_offset_z').value

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = odom_msg.orientation.z
        t.transform.rotation.w = odom_msg.orientation.w

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToLaserBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

