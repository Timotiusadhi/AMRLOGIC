import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DeadReckoningTurtle(Node):
    def __init__(self):
        super().__init__('dead_reckoning_turtle')
        
        # Publisher untuk mengirim perintah gerak
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Timer loop berjalan setiap dt = 0.1 detik (10 Hz)
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.update_position)

        # 1. POSISI AWAL (Asumsi awal berada di tengah kanvas Turtlesim)
        self.x_est = 5.544445
        self.y_est = 5.544445
        self.theta_est = 0.0

        # Kecepatan yang ingin diberikan ke robot
        self.v = 1.0       # linear.x (m/s)
        self.omega = 0.5   # angular.z (rad/s)

        self.get_logger().info("Node Dead Reckoning Aktif! Menghitung posisi mandiri...")

    def update_position(self):
        # A. Buat dan kirim perintah gerak ke simulator
        twist = Twist()
        twist.linear.x = self.v
        twist.angular.z = self.omega
        self.cmd_vel_pub.publish(twist)

        # B. LOGIKA KINEMATIKA: Hitung perkiraan posisi sendiri
        # Update theta (orientasi) terlebih dahulu
        self.theta_est += self.omega * self.dt
        
        # Normalize theta agar nilainya tetap di rentang -PI sampai +PI (opsional)
        self.theta_est = math.atan2(math.sin(self.theta_est), math.cos(self.theta_est))

        # Update koordinat X dan Y berdasarkan theta terbaru
        self.x_est += self.v * math.cos(self.theta_est) * self.dt
        self.y_est += self.v * math.sin(self.theta_est) * self.dt

        # C. Cetak hasil tebakan posisi node ke terminal
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