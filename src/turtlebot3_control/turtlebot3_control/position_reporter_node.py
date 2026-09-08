import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PositionReporter(Node):
    def __init__(self):
        super().__init__('position_reporter')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )
        # Laporkan posisi tiap 1 detik (bukan tiap kali /odom masuk, biar tidak spam)
        self.latest_x = None
        self.latest_y = None
        self.latest_theta = None
        self.report_timer = self.create_timer(5.0, self.report_position)

        self.get_logger().info("Position reporter aktif! Menunggu data /odom...")

    def odom_callback(self, msg):
        self.latest_x = msg.pose.pose.position.x
        self.latest_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.latest_theta = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def report_position(self):
        if self.latest_x is None:
            return
        self.get_logger().info(
            f"Posisi robot -> x: {self.latest_x:.3f} m, "
            f"y: {self.latest_y:.3f} m, "
            f"theta: {self.latest_theta:.3f} rad "
            f"({math.degrees(self.latest_theta):.1f} deg)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PositionReporter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()