import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
import tf2_ros
import serial
import json
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # UART config (chỉnh /dev/ttyUSB0 theo cổng ESP32 của bạn)
        self.ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=0.1)

        # Publisher Odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Timer đọc UART
        self.create_timer(0.05, self.read_uart)  # 20Hz

    def read_uart(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return
            data = json.loads(line)

            # Tạo Odometry msg
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            odom_msg.pose.pose.position.x = data.get("odom_x", 0.0)
            odom_msg.pose.pose.position.y = data.get("odom_y", 0.0)
            odom_msg.pose.pose.position.z = 0.0

            th = data.get("odom_theta", 0.0)
            qz = math.sin(th/2.0)
            qw = math.cos(th/2.0)
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw

            odom_msg.twist.twist.linear.x = data.get("linear_x", 0.0)
            odom_msg.twist.twist.angular.z = data.get("angular_z", 0.0)

            self.odom_pub.publish(odom_msg)

            # Broadcast TF
            t = TransformStamped()
            t.header.stamp = odom_msg.header.stamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = odom_msg.pose.pose.position.x
            t.transform.translation.y = odom_msg.pose.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().warn(f"UART parse error: {e}")

    def cmd_vel_callback(self, msg: Twist):
        # Gửi JSON xuống ESP32
        cmd = {
            "linear": msg.linear.x,
            "angular": msg.angular.z
        }
        try:
            self.ser.write((json.dumps(cmd) + "\n").encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"UART write error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
