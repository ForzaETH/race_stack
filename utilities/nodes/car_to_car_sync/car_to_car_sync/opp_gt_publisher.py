"""Turn the streamed opponent frenet odometry into a ground-truth ObstacleArray.

Replaces perception on the ego car. Both cars must run the same map, so that s/d
refer to the same global waypoints. The opponent pose is broadcast as TF
map -> opp_racecar/base_link, so the sim opponent robot model shows it in rviz.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from f110_msgs.msg import Obstacle, ObstacleArray, WpntArray
from stack_master.qos import LATCHED_QOS


class OppGtPublisher(Node):
    def __init__(self):
        super().__init__('opp_gt_publisher')
        self.size = self.declare_parameter('size', 0.5).value
        self.timeout = self.declare_parameter('timeout', 0.5).value
        rate = self.declare_parameter('rate', 40.0).value
        # /perception/obstacles replaces perception; use e.g. /perception/ground_truth to run alongside it
        obstacles_topic = self.declare_parameter('obstacles_topic', '/perception/obstacles').value
        self.s_max = None
        self.last = None  # (Odometry, rx time)

        self.pub = self.create_publisher(ObstacleArray, obstacles_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, '/opp/car_state/pose', self.pose_cb, 10)
        self.create_subscription(WpntArray, '/global_waypoints', self.wpnts_cb, LATCHED_QOS)
        self.create_subscription(Odometry, '/opp/car_state/frenet/odom', self.odom_cb, 10)
        self.create_timer(1.0 / rate, self.publish)

    def wpnts_cb(self, msg):
        self.s_max = msg.wpnts[-1].s_m

    def odom_cb(self, msg):
        self.last = (msg, self.get_clock().now())

    def pose_cb(self, msg):
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = 'opp_racecar/base_link'
        tf.transform.translation.x = msg.pose.position.x
        tf.transform.translation.y = msg.pose.position.y
        tf.transform.translation.z = msg.pose.position.z
        tf.transform.rotation = msg.pose.orientation
        # front wheels are continuous joints in opp_racecar.xacro, robot_state_publisher needs them from us
        wheels = []
        for side in ('left', 'right'):
            w = TransformStamped()
            w.header.stamp = msg.header.stamp
            w.header.frame_id = f'opp_racecar/front_{side}_hinge'
            w.child_frame_id = f'opp_racecar/front_{side}_wheel'
            w.transform.rotation.w = 1.0
            wheels.append(w)
        self.tf_broadcaster.sendTransform([tf, *wheels])

    def publish(self):
        if self.s_max is None:
            return
        arr = ObstacleArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'frenet'
        if self.last and (self.get_clock().now() - self.last[1]).nanoseconds * 1e-9 < self.timeout:
            odom = self.last[0]
            obs = Obstacle()
            obs.id = 1
            obs.size = self.size
            obs.s_center = odom.pose.pose.position.x
            obs.d_center = odom.pose.pose.position.y
            obs.vs = odom.twist.twist.linear.x
            obs.vd = odom.twist.twist.linear.y
            obs.s_start = (obs.s_center - self.size / 2) % self.s_max
            obs.s_end = (obs.s_center + self.size / 2) % self.s_max
            obs.d_left = obs.d_center + self.size / 2
            obs.d_right = obs.d_center - self.size / 2
            obs.is_static = False
            obs.is_visible = True
            arr.obstacles.append(obs)
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = OppGtPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
