import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        # TARGET dalam koordinat RELATIF terhadap posisi awal robot saat node ini start
        # (bukan koordinat absolut dunia Gazebo)
        self.target_x_rel = 0.5
        self.target_y_rel = 0.5
        self.target_theta_rel = 0.0

        self.state = 'MOVE'

        # Posisi absolut dari /odom (apa adanya, mentah)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.pose_received = False

        # Origin: posisi/orientasi awal robot saat node start — diisi sekali di odom_callback pertama
        self.origin_x = None
        self.origin_y = None
        self.origin_yaw = None

        self.get_logger().info("Node aktif! Menunggu data odometri...")
        self.timer = self.create_timer(0.05, self.control_loop)

    def euler_from_quaternion(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

        if not self.pose_received:
            # Catat posisi SAAT INI sebagai origin (0,0,0) buat run kali ini
            self.origin_x = self.current_x
            self.origin_y = self.current_y
            self.origin_yaw = self.current_yaw
            self.pose_received = True
            self.get_logger().info(
                f"Origin di-set ke posisi saat ini: "
                f"x={self.origin_x:.3f}, y={self.origin_y:.3f}, yaw={self.origin_yaw:.3f}"
            )
            self.get_logger().info("Mulai kalkulasi pergerakan...")

    def control_loop(self):
        if not self.pose_received:
            return

        # Hitung posisi RELATIF terhadap origin (transformasi ke frame origin)
        dx = self.current_x - self.origin_x
        dy = self.current_y - self.origin_y
        cos_o = math.cos(-self.origin_yaw)
        sin_o = math.sin(-self.origin_yaw)
        rel_x = dx * cos_o - dy * sin_o
        rel_y = dx * sin_o + dy * cos_o
        rel_yaw = self.current_yaw - self.origin_yaw
        rel_yaw = math.atan2(math.sin(rel_yaw), math.cos(rel_yaw))

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        distance = math.sqrt(
            (self.target_x_rel - rel_x) ** 2 + (self.target_y_rel - rel_y) ** 2
        )
        angle_to_target = math.atan2(
            self.target_y_rel - rel_y, self.target_x_rel - rel_x
        )

        if self.state == 'MOVE':
            if distance > 0.08:
                angle_diff = angle_to_target - rel_yaw
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                cmd.twist.angular.z = 1.5 * angle_diff
                cmd.twist.linear.x = min(0.22, 0.5 * distance)
            else:
                self.state = 'ROTATE'

        elif self.state == 'ROTATE':
            raw_diff = self.target_theta_rel - rel_yaw
            final_angle_diff = math.atan2(math.sin(raw_diff), math.cos(raw_diff))
            if abs(final_angle_diff) > 0.05:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 1.0 * final_angle_diff
            else:
                self.state = 'DONE'
                self.get_logger().info("Target Berhasil Dicapai!", once=True)

        elif self.state == 'DONE':
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()