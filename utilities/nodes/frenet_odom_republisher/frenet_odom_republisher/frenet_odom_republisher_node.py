import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.node import Node

from rs_msgs.msg import FrenetState, Wpnt, WpntArray
from nav_msgs.msg import Odometry
from frenet_conversion.frenet_converter import FrenetConverter
import numpy as np
from tf_transformations import euler_from_quaternion

# latched: late subscribers still get the last message (publisher and subscriber must both use it)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FrenetOdomRepublisher(Node):

    def __init__(self):
        super().__init__('frenet_odom_republisher')
        self.has_global_trajectory = False
        self.frenet_state_publisher_ = self.create_publisher(FrenetState, '/car_state/frenet', 10)

        self.global_trajectory_sub_ = self.create_subscription(
            WpntArray,
            '/global_waypoints',
            self.global_trajectory_callback,
            LATCHED)
        self.odom_sub_ = self.create_subscription(
            Odometry,
            'car_state/odom',
            self.odom_callback,
            10)

    def global_trajectory_callback(self, msg):
        waypoint_array = msg.wpnts
        waypoints_x = [waypoint.x_m for waypoint in waypoint_array]
        waypoints_y = [waypoint.y_m for waypoint in waypoint_array]
        waypoints_psi = [waypoint.psi_rad for waypoint in waypoint_array]
        self.converter = FrenetConverter(np.array(waypoints_x), np.array(waypoints_y), np.array(waypoints_psi))
        self.has_global_trajectory = True

    def odom_callback(self, msg):
        if self.has_global_trajectory:
            #Odom msg
            odom_pos = msg.pose.pose.position
            odom_quat = msg.pose.pose.orientation
            odom_vel = msg.twist.twist.linear
            theta = euler_from_quaternion([odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w])[2]

            frenet_pos = self.converter.get_frenet([odom_pos.x], [odom_pos.y])
            frenet_vel = self.converter.get_frenet_velocities(odom_vel.x, odom_vel.y, theta)
            
            frenet_msg = FrenetState()
            frenet_msg.header.stamp = msg.header.stamp
            frenet_msg.s = float(frenet_pos[0, 0])
            frenet_msg.d = float(frenet_pos[1, 0])
            frenet_msg.vs = float(frenet_vel[0][0])
            frenet_msg.vd = float(frenet_vel[1][0])
            self.frenet_state_publisher_.publish(frenet_msg)
            

def main(args=None):
    rclpy.init(args=args)

    frenet_odom_republisher = FrenetOdomRepublisher()
    rclpy.spin(frenet_odom_republisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    frenet_odom_republisher.destroy_node()
    rclpy.shutdown()