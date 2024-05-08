import rclpy
from rclpy.node import Node

from f110_msgs.msg import Wpnt, WpntArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from frenet_conversion.frenet_converter import FrenetConverter
import numpy as np
from tf_transformations import euler_from_quaternion


class FrenetOdomRepublisher(Node):

    def __init__(self):
        super().__init__('frenet_odom_republisher')
        self.has_global_trajectory = False
        self.frenet_odom_publisher_ = self.create_publisher(Odometry, '/car_state/frenet/odom', 10)
        self.frenet_pose_publisher_ = self.create_publisher(PoseStamped, '/car_state/frenet/pose', 10)

        self.global_trajectory_sub_ = self.create_subscription(
            WpntArray,
            '/global_waypoints',
            self.global_trajectory_callback,
            10)
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
            
            frenet_msg = msg
            idx = str(self.converter.get_closest_index([odom_pos.x], [odom_pos.y])[0])
            frenet_msg.child_frame_id = idx
            frenet_msg.pose.pose.position.x = frenet_pos[0, 0]
            frenet_msg.pose.pose.position.y = frenet_pos[1, 0]
            frenet_msg.twist.twist.linear.x = frenet_vel[0][0]
            frenet_msg.twist.twist.linear.y = frenet_vel[1][0]
            
            #Pose msg
            frenet_pose_msg = PoseStamped()
            frenet_pose_msg.header = msg.header
            frenet_pose_msg.pose.position.x = frenet_pos[0, 0]
            frenet_pose_msg.pose.position.y = frenet_pos[1, 0]
            
            #Publish or perish
            self.frenet_pose_publisher_.publish(frenet_pose_msg)
            self.frenet_odom_publisher_.publish(frenet_msg)
            

def main(args=None):
    rclpy.init(args=args)

    frenet_odom_republisher = FrenetOdomRepublisher()
    rclpy.spin(frenet_odom_republisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    frenet_odom_republisher.destroy_node()
    rclpy.shutdown()