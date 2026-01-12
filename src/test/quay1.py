#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import RPi.GPIO as GPIO


class StepperNode(Node):
    """
    SIMPLE STEPPER NODE
    - Rotate forward: 0° -> 360°
    - Publish angle while rotating
    - Stop at 360°
    """

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

        # ===== PARAMETERS =====
        self.declare_parameter('pins', [17, 18, 27, 22])
        self.declare_parameter('step_delay', 0.001)
        self.declare_parameter('steps_per_rev', 4096)

        self.pins = list(self.get_parameter('pins').value)
        self.step_delay = float(self.get_parameter('step_delay').value)
        self.steps_per_rev = int(self.get_parameter('steps_per_rev').value)

        # ===== GPIO =====
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for p in self.pins:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, 0)

        # ===== STATE =====
        self.seq_index = 0
        self.current_step = 0  # start at 0°

        # ===== ROS PUB =====
        self.pub_angle = self.create_publisher(Float32, 'lidar_angle', 10)
        self.pub_done  = self.create_publisher(Bool, 'scan_done', 10)

        # ===== TIMER =====
        self.timer = self.create_timer(self.step_delay, self.loop)

        self.get_logger().info('▶️ Stepper rotating 0° → 360°')

    def loop(self):
        if self.current_step >= self.steps_per_rev:
            self.stop_motor()
            self.pub_done.publish(Bool(data=True))
            self.get_logger().info('✅ Reached 360°, motor stopped')
            self.timer.cancel()
            return

        self.apply_step(+1)

        angle = (self.current_step / self.steps_per_rev) * 360.0
        self.pub_angle.publish(Float32(data=float(angle)))

    def apply_step(self, direction: int):
        self.seq_index = (self.seq_index + direction) % 8
        seq = self.SEQ[self.seq_index]
        for pin, val in zip(self.pins, seq):
            GPIO.output(pin, val)
        self.current_step += direction

    def stop_motor(self):
        for p in self.pins:
            GPIO.output(p, 0)

    def destroy_node(self):
        try:
            self.timer.cancel()
        except Exception:
            pass
        self.stop_motor()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StepperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
