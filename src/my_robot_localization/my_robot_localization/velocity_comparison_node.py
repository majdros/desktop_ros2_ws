#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import time

class VelocityComparisonNode(Node):
    def __init__(self):
        super().__init__('velocity_comparison_node')

        # Time and velocity storage
        self.times = []
        self.raw_linear_vel = []
        self.raw_angular_vel = []
        self.filtered_linear_vel = []
        self.filtered_angular_vel = []
        self.imu_angular_vel = []
        self.start_time = time.time()
        
        # Create subscribers
        self.raw_odom_sub = self.create_subscription(Odometry, '/wheel_odom', self.raw_odom_callback, 10)     # Raw odometry topic
        self.raw_imu_sub = self.create_subscription(Imu, '/bno055/imu', self.raw_imu_callback, 10)            # Raw IMU topic
        self.filtered_odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.filtered_odom_callback, 10)  # Filtered odometry topic

        self.get_logger().info('Velocity comparison node started')

        # Timer for plot updates (every 3 seconds)
        self.timer = self.create_timer(3.0, self.plot_data)



    def raw_odom_callback(self, msg):
        try:
            current_time = time.time() - self.start_time
            # Extract linear and angular velocities
            linear_vel = msg.twist.twist.linear.x
            angular_vel = msg.twist.twist.angular.z
            
            # Store data with timestamps
            self.times.append(current_time)
            self.raw_linear_vel.append(linear_vel)
            # self.raw_angular_vel.append(angular_vel)

        except Exception as e:
            self.get_logger().error(f'Error in raw_odom_callback: {str(e)}')



    def raw_imu_callback(self, msg):
        try:
            # Extract angular velocities from IMU
            angular_vel = msg.angular_velocity.z

            # Store IMU angular velocity
            self.imu_angular_vel.append(angular_vel)

        except Exception as e:
            self.get_logger().error(f'Error in raw_imu_callback: {str(e)}')



    def filtered_odom_callback(self, msg):
        try:
            # Extract filtered velocities
            linear_vel = msg.twist.twist.linear.x
            angular_vel = msg.twist.twist.angular.z
            
            # Store filtered data
            self.filtered_linear_vel.append(linear_vel)
            self.filtered_angular_vel.append(angular_vel)

        except Exception as e:
            self.get_logger().error(f'Error in filtered_odom_callback: {str(e)}')



    def plot_data(self):
        # Create figure with two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

        # Plot linear velocities
        ax1.plot(self.times, self.raw_linear_vel, 'b-', label='Raw Odometry')
        if len(self.filtered_linear_vel) > 0:
            # Ensure arrays are the same length for plotting
            min_length = min(len(self.times), len(self.filtered_linear_vel))
            ax1.plot(self.times[:min_length], self.filtered_linear_vel[:min_length], 'r-', label='Filtered')

        ax1.set_title('Linear Velocity Comparison')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Linear Velocity (m/s)')
        ax1.grid(True)
        ax1.legend()

        # Plot angular velocities
        if len(self.filtered_angular_vel) > 0:
            min_length = min(len(self.times), len(self.filtered_angular_vel))
            ax2.plot(self.times[:min_length], self.filtered_angular_vel[:min_length], 'r-', label='Filtered')

        if len(self.imu_angular_vel) > 0:
            min_length = min(len(self.times), len(self.imu_angular_vel))
            ax2.plot(self.times[:min_length], self.imu_angular_vel[:min_length], 'g-', label='IMU')

        ax2.set_title('Angular Velocity Comparison')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Angular Velocity (rad/s)')
        ax2.grid(True)
        ax2.legend()

        plt.tight_layout()

        # Save the plot
        plt.savefig('velocity_comparison.png')
        self.get_logger().info('Plot saved as velocity_comparison.png')

        plt.show()



def main(args=None):
    rclpy.init(args=args)

    node = VelocityComparisonNode()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()