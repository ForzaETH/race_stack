import rclpy
from rclpy.node import Node
import tf2_ros

from geometry_msgs.msg import PoseStamped


class Carstate(Node):
    """Publishes the localized car pose (TF map -> base_link) as /car_state/pose.

    Velocity is on /state_estimation/odom, Frenet state on /car_state/frenet/odom.
    """

    def __init__(self):
        super().__init__('carstate')
        self.declare_parameter('rate_hz', 80.0)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, '/car_state/pose', 10)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.publish_pose)

    def publish_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"map->base_link lookup failed: {e}", throttle_duration_sec=1.0)
            return
        msg = PoseStamped()
        msg.header = trans.header
        msg.pose.position.x = trans.transform.translation.x
        msg.pose.position.y = trans.transform.translation.y
        msg.pose.position.z = trans.transform.translation.z
        msg.pose.orientation = trans.transform.rotation
        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(Carstate())
    rclpy.shutdown()
