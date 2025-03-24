#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from std_msgs.msg import Float32

class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")

        # Subscribe to raw sensor data
        self.odom_sub_ = self.create_subscription(Odometry, "/wheel_odom", self.odomCallback, 10)
        self.imu_sub_ = self.create_subscription(Imu, "/bno055/imu", self.imuCallback, 10)

        # Publish filtered data
        self.odom_pub_ = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.linear_innovation_pub_ = self.create_publisher(Float32, "/kf_innovation/linear", 10)
        self.angular_innovation_pub_ = self.create_publisher(Float32, "/kf_innovation/angular", 10)

        # Zustandsvektor (vx, ω)
        self.mean_ = np.array([[0.0], [0.0]])           # x̂_k|k-1 Zustandsvektor als 2x1 Matrix
        self.variance_ = np.eye(2) * 1000               # P_k|k-1 Kovarianzmatrix als 2x2 Matrix

        # initialize KF_Matrices
        self.A = np.eye(2)                              # Übergangsmatrix
        self.B = np.eye(2)                              # Eingabematrix/Steuermatrix
        self.H = np.eye(2)                              # Messmatrix
        self.I = np.eye(2)                              # Einheitsmatrix

        # sensor values
        self.imu_angular_z_ = 0.0
        self.odom_linear_x_ = 0.0


        self.is_first_odom_ = True
        self.last_angular_z_ = 0.0
        self.last_linear_x_ = 0.0
        self.motion_ = np.array([[0.0], [0.0]])         # u_k-1|k-1 Bewegungsänderung als 2x1 Matrix [Δvx, Δω]

        # Publish the filtered odometry message
        self.kalman_odom_ = Odometry()

        # noise matricses
        self.motion_variance_ = np.eye(2) * 1.5         # Q: Prozessrauschkovarianz für die Reaktionszeit 
        self.measurement_variance_ = np.eye(2) * 1.0    # R: Messrauschkovarianz für die Glättung



    def updateMeasurementNoise(self, innovation):
        """Dynamische Anpassung der Messrauschkovarianz R"""

        error_squared_vx = innovation[0][0] ** 2
        error_squared_w = innovation[1][0] ** 2
        
        # Update both variance components
        self.measurement_variance_[0, 0] = 0.9 * self.measurement_variance_[0, 0] + 0.1 * error_squared_vx
        self.measurement_variance_[1, 1] = 0.9 * self.measurement_variance_[1, 1] + 0.1 * error_squared_w



    def updateProcessNoise(self, motion_rate):
        """Dynamische Anpassung der Prozessrauschkovarianz Q"""
        
        self.motion_variance_[0, 0] = 0.9 * self.motion_variance_[0, 0] + 0.1 * motion_rate[0][0] ** 2
        self.motion_variance_[1, 1] = 0.9 * self.motion_variance_[1, 1] + 0.1 * motion_rate[1][0] ** 2



    def measurementUpdate(self):
        z = np.array([[self.odom_linear_x_], [self.imu_angular_z_]])              # Measurement value (z_k) als 2x1 Matrix
        innovation = z - self.H @ self.mean_                                      # y_k = z_k - H * x̂_k|k-1
        self.linear_innovation_pub_.publish(Float32(data=innovation[0][0]))       # Linear velocity innovation
        self.angular_innovation_pub_.publish(Float32(data=innovation[1][0]))      # Angular velocity innovation

        # Compute Kalman Gain
        S = self.H @ self.variance_ @ self.H.T + self.measurement_variance_       # Vorhersage der Messunsicherheit S_k = H * P_k|k-1*H^T + R
        kalman_gain = self.variance_ @ self.H.T @ np.linalg.inv(S)                # ^

        # Update the state estimate using measurement
        self.mean_ = self.mean_ + kalman_gain @ innovation                        # x̂_k|k = x̂_k|k-1 + K_k * y_k

        # Update the uncertainty
        self.variance_ = (self.I - kalman_gain @ self.H) @ self.variance_         # P_k|k = (I - K_k * H) * P_k|k-1

        self.updateMeasurementNoise(innovation)                                             



    def statePrediction(self):
        # Predict the next state and update uncertainty
        self.mean_ = (self.A  @ self.mean_) + (self.B @ self.motion_)                   # x̂_k|k-1 = A * x̂_k-1|k-1 + B * u_k-1|k-1
        self.variance_ = (self.A  @ self.variance_ @ self.A.T ) + self.motion_variance_ # P_k|k-1 = A * P_k-1|k-1 * A^T + Q

        self.updateProcessNoise(self.motion_)                                               



    def imuCallback(self, imu):
        try:
            # Store the measurement update
            self.imu_angular_z_ = imu.angular_velocity.z

        except Exception as e:
            self.get_logger().error(f'Error in imuCallback: {str(e)}')



    def odomCallback(self, odom):
        try:

            if self.is_first_odom_:
                self.last_linear_x_ = odom.twist.twist.linear.x
                self.last_angular_z_ = odom.twist.twist.angular.z
                self.mean_[0][0] = odom.twist.twist.linear.x
                self.mean_[1][0] = odom.twist.twist.angular.z

                self.is_first_odom_ = False
                return

            # calculate changes for both components
            linear_x_diff = odom.twist.twist.linear.x - self.last_linear_x_
            angular_z_diff = odom.twist.twist.angular.z - self.last_angular_z_

            # update movement change
            self.motion_ = np.array([[linear_x_diff], [angular_z_diff]])

            # Update for the next iteration
            self.last_linear_x_ = odom.twist.twist.linear.x
            self.last_angular_z_ = odom.twist.twist.angular.z

            # Perform state prediction and measurement update
            self.statePrediction()
            self.measurementUpdate()

            self.kalman_odom_ = odom

            # update Header informations
            self.kalman_odom_.header.stamp = self.get_clock().now().to_msg()
            self.kalman_odom_.header.frame_id = "odom"
            self.kalman_odom_.child_frame_id = "base_footprint"

            # Update and publish the filtered linear velocity
            self.kalman_odom_.twist.twist.linear.x = self.mean_[0][0]
            self.kalman_odom_.twist.twist.angular.z = self.mean_[1][0]

            # Publish the updated odometry message
            self.odom_pub_.publish(self.kalman_odom_)


            # broadcast TF transform from odom to base_footprint
            t = TransformStamped()
            t.header.stamp = self.kalman_odom_.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'

            # take position from the odometry message
            t.transform.translation.x = self.kalman_odom_.pose.pose.position.x
            t.transform.translation.y = self.kalman_odom_.pose.pose.position.y 
            t.transform.translation.z = 0.0

            # Use the updated orientation
            t.transform.rotation = self.kalman_odom_.pose.pose.orientation

            # braodcast transform
            TransformBroadcaster(self).sendTransform(t)

        except Exception as e:
            self.get_logger().error(f'Error in odomCallback: {str(e)}')



def main():
    rclpy.init()

    kalman_filter = KalmanFilter()
    rclpy.spin(kalman_filter)
    kalman_filter.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()