#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time

imu_pub = None


def imuCallback(imu):
    global imu_pub
    imu.header.frame_id = "footprint_link_ekf"
    imu_pub.publish(imu)


def main():
    global imu_pub

    rclpy.init()
    node = Node('imu_republisher_node')
    time.sleep(1)

    imu_pub = node.create_publisher(Imu, "imu_ekf", 10)
    imu_sub = node.create_subscription(Imu, "imu/data", imuCallback, 10)
    rclpy.spin(node)

    node.destroy.node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()