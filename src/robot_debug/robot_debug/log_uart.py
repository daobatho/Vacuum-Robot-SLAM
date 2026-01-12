#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import json
import csv
import os
import time

UART_PORT = "/dev/ttyAMA1"   # chỉnh nếu cần
BAUDRATE = 115200

FIELDNAMES = [
    "t",
    "vL_cmd", "vL_fb", "pwmL", "iL",
    "vR_cmd", "vR_fb", "pwmR", "iR"
]


class UARTLogger(Node):
    def __init__(self):
        super().__init__("uart_logger")

        # mở UART
        try:
            self.ser = serial.Serial(
                UART_PORT,
                BAUDRATE,
                timeout=0.05
            )
        except Exception as e:
            self.get_logger().error(f"UART open failed: {e}")
            raise e

        # tạo thư mục log
        log_dir = os.path.expanduser("~/pi_logs")
        os.makedirs(log_dir, exist_ok=True)

        ts = int(time.time())
        self.csv_path = os.path.join(log_dir, f"pi_log_{ts}.csv")

        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=FIELDNAMES
        )
        self.writer.writeheader()

        self.get_logger().info(f"Logging to {self.csv_path}")

        # timer đọc UART
        self.timer = self.create_timer(0.02, self.read_uart)  # 50 Hz

    def read_uart(self):
        try:
            raw = self.ser.readline()
            if not raw:
                return

            # bỏ lỗi decode
            line = raw.decode("utf-8", errors="ignore").strip()

            # chỉ nhận JSON
            if not line.startswith("{"):
                return

            data = json.loads(line)

            # lọc field cho CSV
            row = {k: data.get(k, "") for k in FIELDNAMES}
            self.writer.writerow(row)
            self.csv_file.flush()

        except json.JSONDecodeError:
            pass  # JSON lỗi → bỏ
        except Exception as e:
            self.get_logger().warn(str(e))

    def destroy_node(self):
        self.csv_file.close()
        self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = UARTLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

