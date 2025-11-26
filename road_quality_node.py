import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32


class RoadQualityNode(Node):
    def __init__(self):
        super().__init__("road_quality_node")

        # Subscribe to IMU and Odom
        self.sub_imu = self.create_subscription(Imu, "/imu", self.imu_callback, 50)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Publishers
        self.pub_quality = self.create_publisher(String, "/road_quality", 10)
        self.pub_score = self.create_publisher(Float32, "/road_quality_score", 10)

        # Store last N IMU samples (ax, ay)
        self.window_size = 50      # ~1 second if IMU ~50 Hz
        self.ax_window = deque(maxlen=self.window_size)
        self.ay_window = deque(maxlen=self.window_size)

        self.current_speed = 0.0   # from odom

        # Timer: evaluate quality 10 Hz
        self.timer = self.create_timer(0.1, self.evaluate_quality)

        # Single threshold: below -> GOOD, above -> POOR
        # Smaller number = more sensitive (easier to say POOR)
        self.threshold_poor = 0.85

        self.get_logger().info(
            "RoadQualityNode started. Binary quality: GOOD / POOR (bubble-wrap demo)."
        )

    def imu_callback(self, msg: Imu):
        # Linear acceleration in robot frame
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y

        self.ax_window.append(ax)
        self.ay_window.append(ay)

    def odom_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx * vx + vy * vy)

    def evaluate_quality(self):
        # If not moving, skip (avoid noise when standing still)
        if self.current_speed < 0.05 or len(self.ax_window) < 10:
            return

        # Vibration score = variance(ax) + variance(ay)
        ax_mean = sum(self.ax_window) / len(self.ax_window)
        ay_mean = sum(self.ay_window) / len(self.ay_window)

        var_ax = sum((a - ax_mean) ** 2 for a in self.ax_window) / len(self.ax_window)
        var_ay = sum((a - ay_mean) ** 2 for a in self.ay_window) / len(self.ay_window)

        score = var_ax + var_ay

        # ---- BINARY CLASSIFICATION ----
        if score < self.threshold_poor:
            quality = "GOOD"
        else:
            quality = "POOR"

        # Log + publish
        self.get_logger().info(
            f"ROAD_QUALITY: {quality}  (score={score:.3f}, speed={self.current_speed:.2f} m/s)"
        )

        msg_q = String()
        msg_q.data = quality
        self.pub_quality.publish(msg_q)

        msg_s = Float32()
        msg_s.data = float(score)
        self.pub_score.publish(msg_s)


def main(args=None):
    rclpy.init(args=args)
    node = RoadQualityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
