import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Di bawah threshold ini, velocity dianggap NOL (bukan noise physics engine)
VELOCITY_DEADZONE = 0.01  # rad/s


class JointStateReporter(Node):
    def __init__(self):
        super().__init__('joint_state_reporter')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.latest_msg = None
        self.report_timer = self.create_timer(0.5, self.report_state)
        self.get_logger().info("Joint state reporter aktif! Menunggu data /joint_states...")

    def joint_state_callback(self, msg: JointState):
        self.latest_msg = msg

    def clean(self, value: float) -> float:
        """Terapkan deadzone: nilai kecil (noise physics engine) dibulatkan ke 0."""
        return 0.0 if abs(value) < VELOCITY_DEADZONE else value

    def report_state(self):
        if self.latest_msg is None:
            return

        names = self.latest_msg.name
        positions = self.latest_msg.position
        velocities = self.latest_msg.velocity

        is_moving = any(abs(v) >= VELOCITY_DEADZONE for v in velocities)
        status = "BERGERAK" if is_moving else "DIAM"

        lines = [f"Status robot: {status}"]
        for name, pos, vel in zip(names, positions, velocities):
            clean_vel = self.clean(vel)
            lines.append(
                f"  {name}: posisi={pos:.4f} rad, kecepatan={clean_vel:.4f} rad/s"
            )

        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = JointStateReporter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()