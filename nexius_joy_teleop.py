#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class NexiusTeleop(Node):
    def __init__(self):
        super().__init__('nexius_teleop')

        # Subscribe to joystick and publish cmd_vel
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Base speeds
        self.base_linear = 0.15   # m/s
        self.base_angular = 1.0   # rad/s

        # Last joystick state
        self.last_axes = [0.0, 0.0]
        self.last_buttons = []

        # Celebration spin state
        self.celebration_time_left = 0.0
        self.celebration_speed = 1.5  # rad/s
        self.timer_period = 0.05      # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

    def joy_callback(self, msg: Joy):
        # Save axes for normal driving
        self.last_axes = msg.axes

        # Init last_buttons first time
        if not self.last_buttons:
            self.last_buttons = [0] * len(msg.buttons)

        # Detect button rising edges (0 -> 1)
        for i, val in enumerate(msg.buttons):
            if i >= len(self.last_buttons):
                self.last_buttons.append(0)
            if val == 1 and self.last_buttons[i] == 0:
                self.handle_button_press(i)

        self.last_buttons = list(msg.buttons)

    def handle_button_press(self, idx: int):
        # Button Y triggers celebration spin
        BTN_X = 3

        if idx == BTN_X:
            # 3x 360° celebration spin
            total_angle = 3.0 * 2.0 * math.pi  # 3 full rotations
            self.celebration_time_left = total_angle / self.celebration_speed
            self.get_logger().info('Celebration SPIN! (3x360°)')

    def timer_cb(self):
        twist = Twist()

        if self.celebration_time_left > 0.0:
            # Celebration spin overrides everything
            twist.angular.z = self.celebration_speed
            self.celebration_time_left -= self.timer_period
        else:
            # Normal driving with left stick
            if len(self.last_axes) >= 2:
                # axes[1] = forward/back, axes[0] = left/right (typical)
                linear_input = self.last_axes[1]
                angular_input = self.last_axes[0]

                # Small deadzone
                dead = 0.1
                if abs(linear_input) < dead:
                    linear_input = 0.0
                if abs(angular_input) < dead:
                    angular_input = 0.0

                twist.linear.x = self.base_linear * linear_input
                twist.angular.z = self.base_angular * angular_input

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = NexiusTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
