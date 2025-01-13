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





# class OdomToLaserBroadcaster(Node):
#     def __init__(self):
#         super().__init__('odom_laser_broadcaster')

#         # Transform Broadcaster und Buffer-Listener einrichten
#         self.tf_broadcaster = TransformBroadcaster(self)
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         # Abonnieren des Odometry-Themas
#         self.odom_subscriber = self.create_subscription(
#             Odometry, '/odometry/filtered', self.handle_odom_msg, 50)

#         # Logger-Info
#         self.get_logger().info('Odom to Laser Broadcaster has been started.')

#         # Parameter für den Laser-Offset deklarieren
#         self.declare_parameter('laser_offset_x', 0.0)
#         self.declare_parameter('laser_offset_y', 0.0)
#         self.declare_parameter('laser_offset_z', 0.1955)

#     def handle_odom_msg(self, msg):
#         try:
#             # Abrufen der Transformation zwischen 'odom' und 'laser' (falls vorhanden)
#             transform = self.tf_buffer.lookup_transform(
#                 "odom",  # Ziel-Frame
#                 "laser",  # Quell-Frame
#                 rclpy.time.Time(),  # Aktueller Zeitstempel
#                 timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout für Anfrage
#             )

#             # Transformationsdaten aus dem Buffer übernehmen
#             t = TransformStamped()
#             t.header.stamp = self.get_clock().now().to_msg()
#             t.header.frame_id = transform.header.frame_id
#             t.child_frame_id = transform.child_frame_id

#             t.transform.translation = transform.transform.translation
#             t.transform.rotation = transform.transform.rotation

#         except Exception as e:
#             self.get_logger().warn(f"Transformation nicht gefunden: {e}")
#             # Fallback: Standard-Transformation auf Basis von Offset und Odometry-Daten
#             t = TransformStamped()
#             t.header.stamp = msg.header.stamp
#             t.header.frame_id = 'odom'
#             t.child_frame_id = 'laser'

#             odom_msg = msg.pose.pose
#             t.transform.translation.x = odom_msg.position.x + self.get_parameter('laser_offset_x').value
#             t.transform.translation.y = odom_msg.position.y + self.get_parameter('laser_offset_y').value
#             t.transform.translation.z = self.get_parameter('laser_offset_z').value

#             t.transform.rotation.x = 0.0
#             t.transform.rotation.y = 0.0
#             t.transform.rotation.z = odom_msg.orientation.z
#             t.transform.rotation.w = odom_msg.orientation.w

#         # Transformation publizieren
#         self.tf_broadcaster.sendTransform(t)


# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomToLaserBroadcaster()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
