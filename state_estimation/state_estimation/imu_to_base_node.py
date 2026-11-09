#!/usr/bin/env python3
"""Rotate an IMU message into base_link, synchronously.

Replacement for imu_transformer: that node hands messages through a
tf2_ros::MessageFilter, which publishes queued messages from the TF thread
and fresh ones from the subscription thread, so the output order flips at
startup. Cartographer CHECK-fails on the first out-of-order IMU stamp.

Here the static extrinsic is looked up once, every message is rotated and
published inside its own callback, and anything older than the last
published stamp is dropped.
"""

import numpy as np
import rclpy
import tf2_ros
import tf_transformations as tft
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu


class ImuToBase(Node):

    def __init__(self):
        super().__init__('imu_to_base')
        self.declare_parameter('input_topic', '/sensors/imu/raw')
        self.declare_parameter('output_topic', '/imu')
        self.declare_parameter('target_frame', 'base_link')
        # used when the driver sends zero covariance (vesc_driver does): robot_localization
        # would floor 0 to 1e-9 and trust the sensor absolutely
        self.declare_parameter('gyro_stddev', 0.03)     # rad/s
        self.declare_parameter('accel_stddev', 0.3)     # m/s^2
        # constant gyro offset in the IMU frame, measured at standstill (per car, stack_master/config/<car>/imu.yaml)
        self.declare_parameter('gyro_bias', [0.0, 0.0, 0.0])
        self.gyro_bias = np.array(self.get_parameter('gyro_bias').value, dtype=float)
        # multiplies the bias-corrected rate: integrated gyro yaw / cartographer yaw over whole laps
        self.declare_parameter('gyro_scale', 1.0)
        self.gyro_scale = float(self.get_parameter('gyro_scale').value)

        self.target_frame = self.get_parameter('target_frame').value
        self.R = None          # target <- imu rotation matrix
        self.q_inv = None      # imu <- target quaternion, for orientation
        self.last_stamp_ns = -1

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(
            Imu, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Imu, self.get_parameter('input_topic').value, self._callback, 50)

    def _resolve(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, frame, Time())
        except tf2_ros.TransformException:
            return False
        q = tf.transform.rotation
        q = [q.x, q.y, q.z, q.w]
        self.R = tft.quaternion_matrix(q)[:3, :3]
        self.q_inv = tft.quaternion_inverse(q)
        self.get_logger().info(f'Found extrinsic {self.target_frame} <- {frame}')
        return True

    def _callback(self, msg):
        if self.R is None and not self._resolve(msg.header.frame_id):
            return  # no queue: dropping keeps the output ordered
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp_ns < self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp_ns

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame

        R = self.R
        w = R @ ((np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) - self.gyro_bias)
                 * self.gyro_scale)
        a = R @ [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = map(float, w)
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = map(float, a)
        # R_world_target = R_world_imu * R_imu_target
        q = msg.orientation
        q = tft.quaternion_multiply([q.x, q.y, q.z, q.w], self.q_inv)
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = map(float, q)

        for name, fallback in (('orientation_covariance', None),
                               ('angular_velocity_covariance', 'gyro_stddev'),
                               ('linear_acceleration_covariance', 'accel_stddev')):
            C = np.array(getattr(msg, name)).reshape(3, 3)
            if fallback and not C.any():
                C = np.eye(3) * self.get_parameter(fallback).value ** 2
            setattr(out, name, (R @ C @ R.T).ravel().tolist())

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuToBase()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
