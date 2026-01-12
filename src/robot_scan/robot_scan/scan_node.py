#!/usr/bin/env python3
"""
ROS2 Node: RobotScanNode (fix map đảo 180°)
Ghép dữ liệu TF-Luna + góc quay stepper thành LaserScan cho SLAM Toolbox
Quét -90° → +90°, 1 độ / beam
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Range, LaserScan
import math


class RobotScanNode(Node):
    def __init__(self):
        super().__init__('scan_node')

        # =========================
        # Cấu hình LaserScan
        # =========================
        self.angle_min = -math.pi / 2
        self.angle_max = math.pi / 2
        self.num_beams = 181
        self.angle_inc = (self.angle_max - self.angle_min) / (self.num_beams - 1)

        self.range_min = 0.02
        self.range_max = 8.0
        self.frame_id = 'laser_link'

        # =========================
        # Buffer dữ liệu
        # =========================
        self.ranges = [float('inf')] * self.num_beams
        self.cur_angle = None
        self.updated = False
        self.scan_start_time = None

        # =========================
        # Subscriptions
        # =========================
        self.create_subscription(Float32, 'lidar_angle', self.angle_cb, 10)
        self.create_subscription(Range, 'tf_luna/range', self.range_cb, 50)
        self.create_subscription(Bool, 'scan_done', self.scan_done_cb, 10)

        # =========================
        # Publisher
        # =========================
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.get_logger().info("✅ RobotScanNode (180° fix) started")

    # --------------------------------------------------
    def angle_cb(self, msg: Float32):
        """
        Nhận góc stepper (độ → rad) và đảo 180° map
        """
        self.cur_angle = -math.radians(msg.data)  # <-- đảo dấu 180°

    # --------------------------------------------------
    def range_cb(self, msg: Range):
        if self.cur_angle is None:
            return
        if not (self.range_min <= msg.range <= self.range_max):
            return

        idx = int(round((self.cur_angle - self.angle_min) / self.angle_inc))
        if 0 <= idx < self.num_beams:
            self.ranges[idx] = msg.range
            self.updated = True

    # --------------------------------------------------
    def scan_done_cb(self, msg: Bool):
        if not msg.data:
            return

        # Scan bắt đầu
        if self.scan_start_time is None:
            self.scan_start_time = self.get_clock().now()
            self.get_logger().info("🚀 Scan started")
            return

        # Scan kết thúc
        if not self.updated:
            self.scan_start_time = None
            return

        now = self.get_clock().now()
        elapsed = (now - self.scan_start_time).nanoseconds * 1e-9

        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.scan_time = elapsed
        scan.time_increment = elapsed / self.num_beams
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = list(self.ranges)

        self.scan_pub.publish(scan)

        self.get_logger().info(
            f"📡 Scan published | beams={self.num_beams} | time={elapsed:.2f}s"
        )

        # Reset
        self.ranges = [float('inf')] * self.num_beams
        self.updated = False
        self.scan_start_time = None


def main():
    rclpy.init()
    node = RobotScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
