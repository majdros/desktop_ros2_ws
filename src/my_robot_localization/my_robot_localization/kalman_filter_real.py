#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np

class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kf")

        # Subscribe to raw sensor data
        self.odom_sub_ = self.create_subscription(Odometry, "/wheel_odom", self.odomCallback, 10)
        self.imu_sub_ = self.create_subscription(Imu, "/bno055/imu", self.imuCallback, 10)

        # Publish filtered data
        self.odom_pub_ = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.raw_odom_pub_ = self.create_publisher(Odometry, "raw_odometry/odom", 10)
        self.raw_imu_pub_ = self.create_publisher(Imu, "raw_imu/data", 10)

        # initialize H matrix and I matrix
        self.H = np.array([1.0])                        # Messmatrix (1x1 für Skalare)
        self.A = np.array([1.0])                        # Übergangsmatrix (1x1 für Skalare)
        self.B = np.array([1.0])                        # Eingabematrix bzw. Steuermatrix (1x1 für Skalare)
        self.I = np.eye(1)                              # Einheitsmatrix (1x1 für Skalare)

        self.mean_ = np.array([[0.0]])                  # x̂_k|k-1 als 1x1 Matrix
        self.variance_ = np.array([[1000.0 ]])          # P_k|k-1  als 1x1 Matrix

        self.imu_angular_z_ = 0.0

        self.is_first_odom_ = True
        self.last_angular_z_ = 0.0
        self.motion_ = 0.0                              # u_k-1|k-1

        # Publish the filtered odometry message
        self.kalman_odom_ = Odometry()

        # Modeling the uncertainty of the sensor and the motion
        self.motion_variance_ = np.array([[1.0]])        # Q: Prozessrauschkovarianz als 1x1 Matrix
        self.measurement_variance_ = np.array([[0.5]])   # R: Messrauschkovarianz als 1x1 Matrix



    def updateMeasurementNoise(self, innovation):
        """Dynamische Anpassung der Messrauschkovarianz (R)."""
        error_squared = innovation ** 2
        self.measurement_variance_ = 0.9 * self.measurement_variance_ + 0.1 * error_squared # Glättung



    def updateProcessNoise(self, motion_rate):
        """Dynamische Anpassung der Prozessrauschkovarianz (Q)."""
        self.motion_variance_ = 0.9 * self.motion_variance_ + 0.1 * motion_rate ** 2           # Glättung



    def measurementUpdate(self):
        z = np.array([[self.imu_angular_z_]])                                               # Messwert (z_k) als 1x1 Matrix
        innovation = z - self.H @ self.mean_                                                # y_k = z_k - H * x̂_k|k-1

        # Compute Kalman Gain
        S = self.H @ self.variance_ @ self.H.T + self.measurement_variance_                 # Vorhersage der Messunsicherheit S_k = H * P_k|k-1 * H^T + R
        kalman_gain = self.variance_ @ self.H.T @ np.linalg.inv(S)                          # K_k = P_k|k-1 * H^T / (H * P_k|k-1 * H^T + R)

        # Update the state estimate using measurement
        self.mean_ = self.mean_ + kalman_gain @ innovation                                  # x̂_k|k = x̂_k|k-1 + K_k * y_k

        # Update the uncertainty
        self.variance_ = (self.I - kalman_gain @ self.H) @ self.variance_                   # P_k|k = (I - K_k * H) * P_k|k-1

        self.updateMeasurementNoise(innovation)                                             



    def statePrediction(self):
        # Predict the next state and update uncertainty
        self.mean_ = (self.A  @ self.mean_) + (self.B @ self.motion_)                       # x̂_k|k-1 = A * x̂_k-1|k-1 + B * u_k-1|k-1
        self.variance_ = (self.A  @ self.variance_ @ self.A ) + self.motion_variance_       # P_k|k-1 = A * P_k-1|k-1 * A^T + Q

        self.updateProcessNoise(self.motion_)                                               


    def imuCallback(self, imu):
        # Store the measurement update
        self.imu_angular_z_ = imu.angular_velocity.z

        # Publish the raw IMU data
        self.raw_imu_pub_.publish(imu)



    def odomCallback(self, odom):
        # Publish the raw odometry data
        self.raw_odom_pub_.publish(odom)

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
    kf = KalmanFilter()
    rclpy.spin(kf)
    kf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()