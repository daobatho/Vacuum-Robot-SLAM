#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import RPi.GPIO as GPIO
import time

class StepperNode(Node):
    """
    SLAM-READY STEPPER NODE (WITH TIMING LOG)
    - Scan:   0° -> 360° (publish angle)
    - Return: 360° -> 0° (NO publish angle)
    - Log time for each phase
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

    STATE_SCAN   = 0
    STATE_RETURN = 1

    def __init__(self):
        super().__init__('stepper_node')

        # ===== PARAMETERS =====
        self.declare_parameter('pins', [17, 18, 27, 22])
        self.declare_parameter('step_delay', 0.002)
        self.declare_parameter('return_delay', 0.001)
        self.declare_parameter('steps_per_rev', 4096)

        self.pins = list(self.get_parameter('pins').value)
        self.step_delay = float(self.get_parameter('step_delay').value)
        self.return_delay = float(self.get_parameter('return_delay').value)
        self.steps_per_rev = int(self.get_parameter('steps_per_rev').value)

        # ===== GPIO =====
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for p in self.pins:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, 0)

        # ===== STATE =====
        self.seq_index = 0
        self.current_step = 0
        self.state = self.STATE_SCAN

        # ===== TIME MEASURE =====
        self.scan_start_time = None
        self.return_start_time = None

        # ===== ROS PUB =====
        self.pub_angle = self.create_publisher(Float32, 'lidar_angle', 10)
        self.pub_cycle = self.create_publisher(Bool, 'scan_cycle', 10)

        # ===== TIMER =====
        self.timer = self.create_timer(self.step_delay, self.loop)

        self.get_logger().info('✅ Stepper SLAM-ready node started (timing enabled)')

    # ================= MAIN LOOP =================

    def loop(self):
        if self.state == self.STATE_SCAN:
            self.step_scan()
        else:
            self.step_return()

    # ================= SCAN MODE =================

    def step_scan(self):
        # start timing
        if self.scan_start_time is None:
            self.scan_start_time = time.time()

        self.apply_step(+1)

        angle = (self.current_step / self.steps_per_rev) * 360.0
        self.pub_angle.publish(Float32(data=float(angle)))

        if self.current_step >= self.steps_per_rev:
            scan_time = time.time() - self.scan_start_time
            self.get_logger().info(
                f'🟢 SCAN done: 0° → 360° | time = {scan_time:.3f} s'
            )

            self.pub_cycle.publish(Bool(data=True))

            # switch to return
            self.scan_start_time = None
            self.return_start_time = time.time()
            self.current_step = self.steps_per_rev
            self.state = self.STATE_RETURN

            self.timer.cancel()
            self.timer = self.create_timer(self.return_delay, self.loop)

    # ================= RETURN MODE =================

    def step_return(self):
        self.apply_step(-1)

        if self.current_step <= 0:
            return_time = time.time() - self.return_start_time
            self.get_logger().info(
                f'🔵 RETURN done: 360° → 0° | time = {return_time:.3f} s'
            )

            total_cycle = return_time + (0 if self.scan_start_time else 0)
            self.get_logger().info('🔁 Full scan cycle completed\n')

            # reset state
            self.return_start_time = None
            self.current_step = 0
            self.state = self.STATE_SCAN

            self.timer.cancel()
            self.timer = self.create_timer(self.step_delay, self.loop)

    # ================= LOW LEVEL =================

    def apply_step(self, direction: int):
        self.seq_index = (self.seq_index + direction) % 8
        seq = self.SEQ[self.seq_index]
        for pin, val in zip(self.pins, seq):
            GPIO.output(pin, val)
        self.current_step += direction

    # ================= CLEANUP =================

    def destroy_node(self):
        try:
            self.timer.cancel()
        except Exception:
            pass
        for p in self.pins:
            GPIO.output(p, 0)
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
