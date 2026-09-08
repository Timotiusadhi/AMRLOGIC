import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarReporter(Node):
    def __init__(self):
        super().__init__('lidar_reporter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info("Lidar reporter aktif! Menunggu data /scan...")

    def scan_callback(self, msg: LaserScan):
        detections = []
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and r > 0:
                angle_rad = msg.angle_min + i * msg.angle_increment
                angle_deg = math.degrees(angle_rad)
                detections.append((i, angle_deg, r))

        if not detections:
            self.get_logger().info("Tidak ada objek terdeteksi dalam jangkauan LIDAR.")
            return

        # Cari beam dengan jarak paling dekat (paling relevan buat obstacle terdekat)
        closest_index, closest_angle, closest_range = min(detections, key=lambda d: d[2])

        self.get_logger().info(
            f"Total beam terdeteksi: {len(detections)}/{len(msg.ranges)} | "
            f"Objek TERDEKAT -> beam #{closest_index} "
            f"(sudut {closest_angle:.1f} deg dari depan robot), "
            f"jarak {closest_range:.3f} m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarReporter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()