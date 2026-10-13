#!/usr/bin/env python3
"""Transform Basalt VIO odometry from the camera-IMU body frame to base_link for the EKF."""

import numpy as np
import rclpy
import tf2_ros
import tf_transformations as tft
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class BasaltToEkf(Node):

    def __init__(self):
        super().__init__('basalt_to_ekf')

        self.declare_parameter('input_topic', '/basalt/odomimu')
        self.declare_parameter('output_topic', '/basalt/odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'camera_imu')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('imu_topic', '/camera/imu')
        self.declare_parameter('tf_timeout', 30.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        imu_topic = self.get_parameter('imu_topic').value
        tf_timeout = self.get_parameter('tf_timeout').value

        # Static extrinsic base_link <- camera_imu, read from TF.
        self.T_base_imu = self._lookup_extrinsic(tf_timeout)
        self.T_odom_world = None

        # Basalt's odometry carries no angular velocity, read from the raw IMU.
        self._omega = None
        self._last_warn_time = None

        self.pub = self.create_publisher(Odometry, output_topic, 10)
        self._imu_sub = self.create_subscription(
            Imu, imu_topic, self._imu_callback, 50)
        self._odom_sub = self.create_subscription(
            Odometry, input_topic, self._odom_callback, 10)

    def _lookup_extrinsic(self, tf_timeout):
        # lookup_transform() only resolves once the executor spins the TF
        # subscriptions, so poll with spin_once() during our own timeout
        # window instead of blocking inside a single call.
        buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(buffer, self)
        deadline = self.get_clock().now() + Duration(seconds=tf_timeout)
        last_error = None
        while self.get_clock().now() < deadline:
            try:
                tf_msg = buffer.lookup_transform(
                    self.base_frame, self.imu_frame, Time())
                t = tf_msg.transform.translation
                q = tf_msg.transform.rotation
                T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
                T[:3, 3] = [t.x, t.y, t.z]
                return T
            except tf2_ros.TransformException as e:
                last_error = e
                rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().fatal(
            'BasaltToEkf: extrinsic TF %s<-%s unavailable after %.0fs: %s' %
            (self.base_frame, self.imu_frame, tf_timeout, last_error))
        raise RuntimeError('extrinsic TF lookup failed')

    def _imu_callback(self, msg):
        self._omega = np.array([msg.angular_velocity.x,
                                 msg.angular_velocity.y,
                                 msg.angular_velocity.z])

    def _odom_callback(self, msg):
        omega_imu = self._omega
        if omega_imu is None:
            self.get_logger().warn(
                'BasaltToEkf: no IMU angular velocity yet, skipping',
                throttle_duration_sec=5.0)
            return

        # Pose: world <- base_link
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        T_world_imu = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T_world_imu[:3, 3] = [p.x, p.y, p.z]
        T_world_base = T_world_imu @ tft.inverse_matrix(self.T_base_imu)
        if self.T_odom_world is None:
            self.T_odom_world = tft.inverse_matrix(T_world_base)
            self.get_logger().info('BasaltToEkf: Initialized world frame alignment!')
        T_odom_base = self.T_odom_world @ T_world_base
        p_base = T_odom_base[:3, 3]
        q_base = tft.quaternion_from_matrix(T_odom_base)

        # Velocity: rotate into base_link and add the lever-arm term.
        R_base_imu = self.T_base_imu[:3, :3]
        t_base_imu = self.T_base_imu[:3, 3]
        v = msg.twist.twist.linear
        omega_base = R_base_imu @ omega_imu
        v_base = R_base_imu @ np.array([v.x, v.y, v.z]) - np.cross(omega_base, t_base_imu)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.world_frame
        out.child_frame_id = self.base_frame

        out.pose.pose.position.x = float(p_base[0])
        out.pose.pose.position.y = float(p_base[1])
        out.pose.pose.position.z = float(p_base[2])
        out.pose.pose.orientation.x = float(q_base[0])
        out.pose.pose.orientation.y = float(q_base[1])
        out.pose.pose.orientation.z = float(q_base[2])
        out.pose.pose.orientation.w = float(q_base[3])

        out.twist.twist.linear.x = float(v_base[0])
        out.twist.twist.linear.y = float(v_base[1])
        out.twist.twist.linear.z = float(v_base[2])
        out.twist.twist.angular.x = float(omega_base[0])
        out.twist.twist.angular.y = float(omega_base[1])
        out.twist.twist.angular.z = float(omega_base[2])

        out.pose.covariance[0] = 0.05    # x
        out.pose.covariance[7] = 0.05    # y
        out.pose.covariance[35] = 0.01   # yaw
        out.twist.covariance[0] = 0.005  # vx
        out.twist.covariance[7] = 0.005  # vy

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BasaltToEkf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
