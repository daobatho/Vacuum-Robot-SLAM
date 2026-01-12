import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import threading
import queue
import time

DEFAULT_PORT = "/dev/ttyAMA2"  # điều chỉnh nếu cần
DEFAULT_BAUD = 115200

class TFLunaReader(threading.Thread):
    def __init__(self, port, baud, out_q):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.q = out_q
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
        except Exception as e:
            raise RuntimeError(f"Cannot open serial {self.port}: {e}")
        # flush
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        # continuous read & frame sync
        while not self._stop.is_set():
            b = self.ser.read(1)
            if not b:
                continue
            if b != b'\x59':
                continue
            b2 = self.ser.read(1)
            if b2 != b'\x59':
                continue
            rest = self.ser.read(7)
            if len(rest) != 7:
                continue
            frame = b + b2 + rest
            # checksum validation
            cs = sum(frame[0:8]) & 0xFF
            if cs != frame[8]:
                # bad frame -> skip
                continue
            dist = frame[2] + frame[3]*256
            strength = frame[4] + frame[5]*256
            temp_raw = frame[6] + frame[7]*256
            temp_c = (temp_raw / 8.0) - 256.0
            # push to queue (non-blocking)
            try:
                self.q.put_nowait((dist, strength, temp_c, time.time()))
            except queue.Full:
                # drop if full
                pass

class TFLunaNode(Node):
    def __init__(self):
        super().__init__('tf_luna_node')
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baud', DEFAULT_BAUD)
        self.declare_parameter('frame_id', 'tf_luna_link')
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').get_parameter_value().integer_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pub = self.create_publisher(Range, 'tf_luna/range', 10)
        self.q = queue.Queue(maxsize=500)

        try:
            self.reader = TFLunaReader(port, baud, self.q)
        except RuntimeError as e:
            self.get_logger().error(str(e))
            raise

        self.reader.start()
        # Timer: publish at up to 200Hz (adjust if desired)
        self.create_timer(0.005, self.timer_cb)
        self.get_logger().info(f"TF-Luna node started on {port} @ {baud}")

    def timer_cb(self):
        # publish latest available frame (if any)
        try:
            item = self.q.get_nowait()
        except queue.Empty:
            return
        dist_cm, strength, temp_c, ts = item
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED
        # convert cm -> m
        msg.range = float(dist_cm) / 100.0
        msg.min_range = 0.02
        msg.max_range = 8.0
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.reader.stop()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TFLunaNode()
        rclpy.spin(node)
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
