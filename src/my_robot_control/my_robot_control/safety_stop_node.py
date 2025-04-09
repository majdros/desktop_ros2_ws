#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# from my_robot_interfaces.msg import CustomBool
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from pynput import keyboard


class SafetyStopNode(Node):
    def __init__(self):
        super().__init__('safety_stop_node')

        self.emergency_stop_active = False
        self.last_emergency_stop_state = None

        self.emergency_stop_publisher = self.create_publisher(Bool, '/emergency_stop', 10)

        self.declare_parameter('use_teleop_joy', False)
        self.use_teleop_joy = self.get_parameter('use_teleop_joy').get_parameter_value().bool_value

        if self.use_teleop_joy:
            self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.declare_parameter('use_teleop_keyboard', True)
        self.use_teleop_keyboard = self.get_parameter('use_teleop_keyboard').get_parameter_value().bool_value
        
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.publish_emergency_stop_state)


    def publish_emergency_stop_state(self):
        if self.last_emergency_stop_state != self.emergency_stop_active:
            self.get_logger().info(f'Emergency stop state is:: {self.emergency_stop_active}')
            self.display_message()
            self.last_emergency_stop_state = self.emergency_stop_active

        msg = Bool()
        msg.data = self.emergency_stop_active
        self.emergency_stop_publisher.publish(msg)


    def on_press(self, key):
        try:
            if key == keyboard.Key.insert:
                self.activate_emergency_stop()
            elif key == keyboard.Key.delete:
                self.release_emergency_stop()
        except AttributeError:
            pass  # Ignore special keys


    def joy_callback(self, msg):
        try:
            # Use Button 0 (BACK) as the emergency stop button
            if msg.buttons and msg.buttons[6] == 1:
                if not self.emergency_stop_active:
                    self.activate_emergency_stop()

            # Use Button 7 (START) as the emergency release button
            elif msg.buttons and msg.buttons[7] == 1:
                if self.emergency_stop_active:
                    self.release_emergency_stop()
        except Exception as e:
            self.get_logger().error(f'Error in joy_callback: {str(e)}')


    def activate_emergency_stop(self):
        self.emergency_stop_active = True
        self.get_logger().info('Emergency_stop ACTIVATED!')
        self.display_message()


    def release_emergency_stop(self):
        if self.emergency_stop_active:
            self.emergency_stop_active = False
            self.get_logger().info('Emergency_stop released')
        else:
            self.get_logger().info('Emergency_stop is already released!')
            self.display_message()


    def display_message(self):
        if self.use_teleop_keyboard:
            self.get_logger().info("🚨 Press 'insert' to activate the emergency stop, or 'delete' to reset it. 🚨")
        if self.use_teleop_joy:
            self.get_logger().info("🚨 Press 'Button 0 (BACK)' to activate the emergency stop, or 'Button 7 (START)' to reset it. 🚨")


def main(args=None):
    rclpy.init(args=args)

    node = SafetyStopNode()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()