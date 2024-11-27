#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")

        # self.odom_sub_ = self.create_subscription(Odometry, "/wheel_odom", self.odomCallback, 10)     # echter Sensor (Encoder)
        self.odom_sub_ = self.create_subscription(Odometry, "noisy_odometry/odom_noisy", self.odomCallback, 10)   # plugin

        self.imu_sub_ = self.create_subscription(Imu, "imu/data", self.imuCallback, 10)   # plugin
        # self.imu_sub_ = self.create_subscription(Imu, "/bno055/imu", self.imuCallback, 10)  # echter Sensor (IMU)
        self.odom_pub_ = self.create_publisher(Odometry, "noisy_odometry/odom_kalman", 10)

        self.mean_ = 0.0
        self.variance_ = 1000.0

        # Store the messages - only for the orientation
        self.imu_angular_z_ = 0.0

        self.is_first_odom_ = True
        self.last_angular_z_ = 0.0
        self.motion_ = 0.0

        # Publish the filtered odometry message
        self.kalman_odom_ = Odometry()

        # Modeling the uncertainty of the sensor and the motion
        self.motion_variance_ = 4.0
        self.measurement_variance_ = 0.5




    def measurementUpdate(self):
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z_) / (self.variance_ + self.measurement_variance_)
        self.variance_ = (self.variance_ * self.measurement_variance_) / (self.variance_ + self.measurement_variance_)

    def statePrediction(self):
        self.mean_ = self.mean_ + self.motion_
        self.variance_ = self.variance_ + self.motion_variance_



    def imuCallback(self, imu):
        # Store the measurement update
        self.imu_angular_z_ = imu.angular_velocity.z

    def odomCallback(self, odom):
        self.kalman_odom_ = odom

        if self.is_first_odom_:
            self.last_angular_z_ = odom.twist.twist.angular.z
            self.mean_ = odom.twist.twist.angular.z

            self.is_first_odom_ = False
            return
        
        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_

        # Update for the next iteration
        self.last_angular_z_ = odom.twist.twist.angular.z

        # Perform state prediction and measurement update
        self.statePrediction()
        self.measurementUpdate()

        # Update and publish the filtered odom message
        self.kalman_odom_.twist.twist.angular.z = self.mean_
        self.odom_pub_.publish(self.kalman_odom_)


def main():
    rclpy.init()
    kalman_filter = KalmanFilter()
    rclpy.spin(kalman_filter)
    kalman_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()