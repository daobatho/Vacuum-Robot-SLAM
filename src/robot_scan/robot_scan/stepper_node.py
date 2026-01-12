#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import RPi.GPIO as GPIO

class StepperNode(Node):
    # Half-step sequence cho 28BYJ-48
    SEQ = [
        [1,0,0,0],
        [1,1,0,0],
        [0,1,0,0],
        [0,1,1,0],
        [0,0,1,0],
        [0,0,1,1],
        [0,0,0,1],
        [1,0,0,1]
    ]

    def __init__(self):
        super().__init__('stepper_node')

        # GPIO BCM
        self.pins = [17, 18, 27, 22]

        # ===== SCAN CONFIG =====
        self.delay = 0.001          # chu kỳ timer (1 ms)
        self.steps_180 = 2048       # 180° full resolution

        # ===== GPIO INIT =====
        GPIO.setmode(GPIO.BCM)
        for p in self.pins:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, 0)

        # ===== STATE =====
        self.seq_idx = 0
        self.step = 0
        self.dir = 1  # 1: tiến, -1: lùi

        # ===== ROS PUB =====
        self.angle_pub = self.create_publisher(Float32, 'lidar_angle', 10)
        self.scan_done_pub = self.create_publisher(Bool, 'scan_done', 10)

        # ===== TIMER =====
        self.timer = self.create_timer(self.delay, self.loop)

        self.get_logger().info("✅ StepperNode READY (ROS2-timer driven, no sleep)")

    def loop(self):
        self.apply_step(self.dir)

        # Tính góc (-90° → +90°)
        angle = (self.step / self.steps_180) * 180.0 - 90.0
        self.angle_pub.publish(Float32(data=angle))

        # Đảo chiều khi chạm biên
        if self.step >= self.steps_180:
            self.dir = -1
            self.scan_done_pub.publish(Bool(data=True))
        elif self.step <= 0:
            self.dir = 1
            self.scan_done_pub.publish(Bool(data=True))

    def apply_step(self, direction):
        self.seq_idx = (self.seq_idx + direction) % 8
        for pin, val in zip(self.pins, self.SEQ[self.seq_idx]):
            GPIO.output(pin, val)
        self.step += direction

    def destroy_node(self):
        for p in self.pins:
            GPIO.output(p, 0)
        GPIO.cleanup()
        super().destroy_node()

def main():
    rclpy.init()
    node = StepperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
