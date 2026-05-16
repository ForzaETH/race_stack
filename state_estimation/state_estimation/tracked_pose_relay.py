import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class TrackedPoseRelay(Node):
    """
    Relays /tracked_pose (geometry_msgs/PoseStamped from Cartographer) to
    /tracked_pose_with_cov (geometry_msgs/PoseWithCovarianceStamped) by attaching
    a fixed diagonal covariance representing expected Cartographer localization uncertainty.

    robot_localization requires PoseWithCovarianceStamped for pose inputs, and the
    covariance values here determine how much EKF2 trusts each Cartographer correction
    relative to its odometry prediction. Tune position_variance and yaw_variance based
    on observed Cartographer accuracy on your specific track and map.
    """

    def __init__(self):
        super().__init__('tracked_pose_relay',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # Covariance values for the Cartographer pose estimate.
        # These are variances (sigma^2), not standard deviations.
        # - position_variance: m^2. Default 0.05 → σ ≈ 22 cm.
        # - yaw_variance: rad^2. Default 0.05 → σ ≈ 12.7 deg.
        # Decrease to trust Cartographer more; increase to rely more on odometry.
        self.declare_parameter('position_variance', 0.05)
        self.declare_parameter('yaw_variance', 0.05)

        self.pos_var = self.get_parameter('position_variance').value
        self.yaw_var = self.get_parameter('yaw_variance').value

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/tracked_pose_with_cov', 10)
        self.sub = self.create_subscription(PoseStamped, '/tracked_pose', self.cb, 10)

        self.get_logger().info(
            f"tracked_pose_relay started: position_variance={self.pos_var}, yaw_variance={self.yaw_var}"
        )

    def cb(self, msg: PoseStamped):
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose.pose = msg.pose

        # Diagonal covariance for [x, y, z, roll, pitch, yaw] in row-major 6x6 layout.
        # Off-diagonal entries stay zero (no cross-correlations assumed).
        out.pose.covariance[0]  = self.pos_var   # x variance
        out.pose.covariance[7]  = self.pos_var   # y variance
        out.pose.covariance[14] = 1e-9           # z  (2D mode: near-zero)
        out.pose.covariance[21] = 1e-9           # roll  (2D mode: near-zero)
        out.pose.covariance[28] = 1e-9           # pitch (2D mode: near-zero)
        out.pose.covariance[35] = self.yaw_var   # yaw variance

        self.pub.publish(out)


def main():
    rclpy.init()
    node = TrackedPoseRelay()
    rclpy.spin(node)
    rclpy.shutdown()
