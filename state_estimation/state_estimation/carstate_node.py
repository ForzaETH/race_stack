import rclpy
from rclpy.node import Node
import tf2_ros

import time
import numpy as np

from sensor_msgs.msg import Imu
from f110_msgs.msg import WpntArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from frenet_conversion.frenet_converter import FrenetConverter
from tf_transformations import euler_from_quaternion

# Carstate node is relevant for SE1 only (basic state estimaiton pipeline).
# The carstate node publishes the "final" state estimation of the car, in this case the velocities from the (early fusion) EKF and the pose from localization (by default cartographer SLAM).

class Carstate(Node):
    def __init__(self):
        super().__init__('carstate',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.get_logger().info("Carstate node started")

        # ros params
        self.declare_parameter('/carstate_node/odom_topic', "/early_fusion/odom")
        self.declare_parameter('/carstate_node/odom_out_topic', "/car_state/odom")
        self.declare_parameter('/carstate_node/pose_out_topic', "/car_state/pose")
        self.odom_in_topic = self.get_parameter('/carstate_node/odom_topic').value
        self.odom_out_topic = self.get_parameter('/carstate_node/odom_out_topic').value
        self.pose_out_topic = self.get_parameter('/carstate_node/pose_out_topic').value
        self.frenet_bool = self.get_parameter('frenet_bool').get_parameter_value().bool_value # Bool if we want frenet on or off

        # Wait until the requried tf transforms exist
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # data containers
        self.ekf_odom = None
        self.frenet_converter = None
        self.gb_wpnts = None
        self.car_state_odom = None

        # subscribers
        self.ekf_odom_sub = self.create_subscription(Odometry, self.odom_in_topic, self.ekf_odom_cb, 10)
        self.gb_wpnts_sub = self.create_subscription(WpntArray, "/global_waypoints", self.gb_wpnts_cb, 10)

        # publishers
        self.state_odom_pub = self.create_publisher(Odometry, self.odom_out_topic, 10)
        self.state_pose_pub = self.create_publisher(PoseStamped, self.pose_out_topic, 10)
        self.frenet_state_odom_pub = self.create_publisher(Odometry, "/car_state/frenet/odom", 10)
        self.frenet_state_pose_pub = self.create_publisher(PoseStamped, "/car_state/frenet/pose", 10)

        # Block until relevant data is here
        self.wait_for_messages(frenet_bool=self.frenet_bool)

        # Publish at 80 Hz
        self.create_timer(1/80, self.cartesian_state_loop)
        if self.frenet_bool:
            self.create_timer(1/80, self.frenet_state_loop)

    def ekf_odom_cb(self, data):
        self.ekf_odom = data

    def gb_wpnts_cb(self, data):
        self.gb_wpnts = data

    def get_slam_tf(self) -> TransformStamped:
        trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), rclpy.duration.Duration(seconds=6.9))
        return trans


    def wait_for_messages(self, frenet_bool: bool = False):
        self.get_logger().info('Carstate Node waiting for Odometry messages...')
        ekf_print = False
        frenet_print = False
        while self.ekf_odom is None or (frenet_bool and self.gb_wpnts is None):
            rclpy.spin_once(self)
            if self.ekf_odom is not None and not ekf_print:
                self.get_logger().info('Received Odometry message.')
                ekf_print = True
            if frenet_bool and self.gb_wpnts is not None and not frenet_print:
                waypoint_array = self.gb_wpnts.wpnts
                waypoints_x = [waypoint.x_m for waypoint in waypoint_array]
                waypoints_y = [waypoint.y_m for waypoint in waypoint_array]
                waypoints_psi = [waypoint.psi_rad for waypoint in waypoint_array]
                self.frenet_converter = FrenetConverter(np.array(waypoints_x), np.array(waypoints_y), np.array(waypoints_psi))
                self.get_logger().info('Received Global Waypoints message and frenet converter initialized!')
                frenet_print = True
        self.get_logger().info('All required messages received. Continuing...')


    def cartesian_state_loop(self):
        # publish SLAM cartesian positional and velocity data
        carstate_pose_msg = PoseStamped()
        carstate_odom_msg = Odometry()

        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), rclpy.duration.Duration(seconds=6.9))
        except Exception as e:
            self.get_logger().warn(f"{e}")
            return

        # build pose message
        carstate_pose_msg.header = trans.header
        carstate_pose_msg.pose.position.x = trans.transform.translation.x
        carstate_pose_msg.pose.position.y = trans.transform.translation.y
        carstate_pose_msg.pose.position.z = trans.transform.translation.z
        carstate_pose_msg.pose.orientation = trans.transform.rotation

        # build odometry message
        carstate_odom_msg.header = carstate_pose_msg.header
        carstate_odom_msg.pose.pose = carstate_pose_msg.pose

        # handle speed from EKF
        carstate_odom_msg.twist.twist = self.ekf_odom.twist.twist  # Make sure to use twist.twist
        self.car_state_odom = carstate_odom_msg

        # publish
        self.state_odom_pub.publish(carstate_odom_msg)
        self.state_pose_pub.publish(carstate_pose_msg)


    def frenet_state_loop(self):
        if self.car_state_odom is None:
            return

        odom_cart = self.car_state_odom
        x_cart = odom_cart.pose.pose.position.x
        y_cart = odom_cart.pose.pose.position.y
        vx = odom_cart.twist.twist.linear.x
        vy = odom_cart.twist.twist.linear.y
        q_cart = odom_cart.pose.pose.orientation
        theta = euler_from_quaternion([q_cart.x, q_cart.y, q_cart.z, q_cart.w])[2]

        # get frenet coordinates and velocities
        frenet_pos = self.frenet_converter.get_frenet([x_cart], [y_cart])
        frenet_vel = self.frenet_converter.get_frenet_velocities(vx, vy, theta)

        s = frenet_pos[0, 0]
        d = frenet_pos[1, 0]
        vs = frenet_vel[0][0]
        vd = frenet_vel[1][0]

        #frenet pose msg
        frenet_pose_msg = PoseStamped()
        frenet_pose_msg.header.stamp = odom_cart.header.stamp
        frenet_pose_msg.header.frame_id = "frenet"
        frenet_pose_msg.pose.position.x = s
        frenet_pose_msg.pose.position.y = d

        #frenet odom msg
        frenet_odom_msg = Odometry()
        frenet_odom_msg.header = frenet_pose_msg.header
        frenet_odom_msg.pose.pose = frenet_pose_msg.pose
        frenet_odom_msg.twist.twist.linear.x = vs
        frenet_odom_msg.twist.twist.linear.y = vd
        #TODO handle speed from EKF

        #publish
        self.frenet_state_odom_pub.publish(frenet_odom_msg)
        self.frenet_state_pose_pub.publish(frenet_pose_msg)

def main():
    rclpy.init()
    node = Carstate()
    rclpy.spin(node)
    rclpy.shutdown()
