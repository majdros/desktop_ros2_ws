#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class NoisyOdometryPublisher(Node):
    def __init__(self):
        super().__init__('noisy_odometry_publisher')
        
        # Robot-spezifische Parameter aus URDF
        self.wheel_radius = 0.0325  # wheel_radius aus URDF
        self.wheel_separation = 0.15  # 2 * wheel_y_offset aus URDF
        
        # Publisher für verrauschte Odometriedaten
        self.noisy_odom_pub = self.create_publisher(
            Odometry,
            'noisy_odometry/odom_noisy',
            10
        )
        
        # Subscriber für cmd_vel (falls vorhanden)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'diff_cont/cmd_vel_unstamped',
            self.cmd_vel_callback,
            10
        )
        
        # Timer für regelmäßige Updates
        self.update_rate = 100.0  # Hz
        self.create_timer(1.0/self.update_rate, self.timer_callback)
        
        # Initialisierung der Odometrie-Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Aktuelle Geschwindigkeiten
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # Parameter für das Rauschen
        # Angepasst an die Größe und Masse des Roboters
        self.linear_velocity_noise_std = 0.02  # 2cm/s Standardabweichung
        self.angular_velocity_noise_std = 0.05  # 0.05 rad/s Standardabweichung
        
        # Maximale Geschwindigkeiten basierend auf Robotergeometrie
        self.max_linear_velocity = 0.5  # m/s
        self.max_angular_velocity = 2.0  # rad/s
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Simulierte Encoder-Fehler
        self.right_wheel_error = 0.02  # 2% Fehler
        self.left_wheel_error = 0.02   # 2% Fehler

    def cmd_vel_callback(self, msg):
        """Callback für cmd_vel Nachrichten"""
        self.current_linear_vel = min(msg.linear.x, self.max_linear_velocity)
        self.current_angular_vel = min(msg.angular.z, self.max_angular_velocity)

    def add_noise(self, value, std_dev):
        """Fügt Gauß'sches Rauschen zu einem Wert hinzu"""
        return value + np.random.normal(0, std_dev)

    def simulate_wheel_velocities(self):
        """Simuliert die Radgeschwindigkeiten mit Encoder-Fehlern"""
        # Berechne ideale Radgeschwindigkeiten
        v_r = (self.current_linear_vel + self.wheel_separation * self.current_angular_vel / 2.0)
        v_l = (self.current_linear_vel - self.wheel_separation * self.current_angular_vel / 2.0)
        
        # Füge Encoder-Fehler hinzu
        v_r *= (1.0 + np.random.normal(0, self.right_wheel_error))
        v_l *= (1.0 + np.random.normal(0, self.left_wheel_error))
        
        return v_r, v_l

    def timer_callback(self):
        # Simuliere Radgeschwindigkeiten mit Fehlern
        v_r, v_l = self.simulate_wheel_velocities()
        
        # Berechne verrauschte Geschwindigkeiten
        noisy_linear_vel = (v_r + v_l) / 2.0
        noisy_angular_vel = (v_r - v_l) / self.wheel_separation
        
        # Füge zusätzliches Sensorrauschen hinzu
        noisy_linear_vel = self.add_noise(noisy_linear_vel, self.linear_velocity_noise_std)
        noisy_angular_vel = self.add_noise(noisy_angular_vel, self.angular_velocity_noise_std)
        
        # Zeitschritt
        dt = 1.0 / self.update_rate
        
        # Update Position und Orientierung
        self.x += noisy_linear_vel * math.cos(self.theta) * dt
        self.y += noisy_linear_vel * math.sin(self.theta) * dt
        self.theta += noisy_angular_vel * dt
        
        # Normalisiere Theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Erstelle Odometry-Nachricht
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'body_link'
        
        # Setze Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Setze Orientierung (als Quaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta/2)
        odom.pose.pose.orientation.w = math.cos(self.theta/2)
        
        # Setze Geschwindigkeiten
        odom.twist.twist.linear.x = noisy_linear_vel
        odom.twist.twist.angular.z = noisy_angular_vel
        
        # Füge Kovarianzmatrizen hinzu
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[35] = 0.01 # theta
        odom.twist.covariance[0] = 0.01 # linear velocity
        odom.twist.covariance[35] = 0.01 # angular velocity
        
        # Publiziere Odometry
        self.noisy_odom_pub.publish(odom)
        
        # Broadcast TF
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = NoisyOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()