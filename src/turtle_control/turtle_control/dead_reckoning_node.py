import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DeadReckoningTurtle(Node):
    def __init__(self):
        super().__init__('dead_reckoning_turtle')
        
        # Publisher untuk perintah gerak
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Timer loop dt = 0.1 detik (10 Hz)
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.update_position)

        # Posisi Awal (Tebakan/Hardcoded)
        self.x_est = 5.544445
        self.y_est = 5.544445
        self.theta_est = 0.0

        # Kecepatan yang dikirim
        self.v = 2.0       # linear.x (m/s)
        self.omega = 0.0   # angular.z (rad/s)

        self.get_logger().info("Node Dead Reckoning Aktif! Menghitung posisi mandiri...")

    def update_position(self):
        # Kirim perintah gerak
        twist = Twist()
        twist.linear.x = self.v
        twist.angular.z = self.omega
        self.cmd_vel_pub.publish(twist)

        # Hitung posisi sendiri (Dead Reckoning)
        self.theta_est += self.omega * self.dt
        self.theta_est = math.atan2(math.sin(self.theta_est), math.cos(self.theta_est))

        self.x_est += self.v * math.cos(self.theta_est) * self.dt
        self.y_est += self.v * math.sin(self.theta_est) * self.dt

        self.get_logger().info(
            f"Estimasi Posisi -> X: {self.x_est:.2f}, Y: {self.y_est:.2f}, Theta: {self.theta_est:.2f} rad"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningTurtle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()