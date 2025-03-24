#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import time

class InnovationEvaluator(Node):
    def __init__(self):
        super().__init__('innovation_evaluator')
        
        # Time and innovation storage
        self.times = []
        self.linear_innovations = []
        self.angular_innovations = []
        self.start_time = time.time()
        
        # Create subscribers
        self.linear_innovation_sub = self.create_subscription(Float32, '/kf_innovation/linear', self.linear_innovation_callback, 10)
        self.angular_innovation_sub = self.create_subscription(Float32, '/kf_innovation/angular', self.angular_innovation_callback, 10)

        self.get_logger().info('Innovation evaluator node started')
        
        # Timer for plot updates (every 3 seconds)
        self.timer = self.create_timer(3.0, self.plot_data)



    def linear_innovation_callback(self, msg):
        current_time = time.time() - self.start_time
        self.times.append(current_time)
        self.linear_innovations.append(msg.data)



    def angular_innovation_callback(self, msg):
        self.angular_innovations.append(msg.data)



    def plot_data(self):
        # Create figure with two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Plot linear innovations
        ax1.plot(self.times, self.linear_innovations, 'b-', label='Linear Innovation')
        ax1.set_title('Linear Innovation Over Time')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Linear Innovation (m/s)')
        ax1.grid(True)
        ax1.legend()
        
        # Plot angular innovations
        ax2.plot(self.times, self.angular_innovations, 'r-', label='Angular Innovation')
        ax2.set_title('Angular Innovation Over Time')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Angular Innovation (rad/s)')
        ax2.grid(True)
        ax2.legend()
        
        plt.tight_layout()
        
        # Save the plot
        plt.savefig('innovation_comparison.png')
        self.get_logger().info('Plot saved as innovation_comparison.png')


        plt.show()

def main(args=None):
    rclpy.init(args=args)

    node = InnovationEvaluator()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()