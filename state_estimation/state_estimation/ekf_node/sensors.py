#!/usr/bin/env python3
"""
Reusable sensor abstraction for the EKF node.

Each sensor owns its subscription, buffers the newest measurement, and knows how
to apply its own sequential EKF update using the model's measurement functions.
Adding a new sensor to the filter is a single line in the node (see EkfNode).
"""
import time

import numpy as np
import tf_transformations as tft
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def normalize_angle(x):
    x = x % (2 * np.pi)
    if x > np.pi:
        x -= 2 * np.pi
    return x


class Sensor:
    """
    Base class for a measurement source feeding the EKF.

    Subclasses only need to implement ``parse(msg)``, which reads a ROS message
    into ``self.data``. Everything else (subscription, buffering, freshness
    tracking, the EKF update and its timing stats) is handled here.

    Args:
        node:          owning EkfNode (used for the subscription and the shared lock)
        name:          short identifier, used for logging / performance reports
        msg_type:      ROS message type to subscribe to
        topic:         topic name
        dim:           measurement vector length
        R:             measurement covariance matrix
        Hx:            model measurement function h(x[, u])
        HJacobian:     model measurement Jacobian H(x[, u])
        uses_control:  if True, the control input u is passed to Hx/HJacobian
        angle_idx:     index in the measurement that is an angle (wrapped in the
                       residual), or None if there is no angular measurement
        relative:      if True, every measurement is re-expressed relative to this
                       sensor's own first measurement, so it starts at zero and
                       contributes no absolute reference to the filter. Requires
                       the subclass to implement ``rebase``.
        on_init:       optional callable invoked with the first measurement (after
                       ``rebase``), used to seed the filter state.
    """

    def __init__(self, node, name, msg_type, topic, dim, R, Hx, HJacobian,
                 uses_control=False, angle_idx=None, relative=False, on_init=None):
        self.node = node
        self.name = name
        self.R = R
        self.Hx = Hx
        self.HJacobian = HJacobian
        self.uses_control = uses_control
        self.angle_idx = angle_idx
        self.relative = relative
        self.on_init = on_init

        self.data = np.zeros(dim)
        self.fresh = False
        self._origin = None
        self._seeded = False

        # Performance stats
        self.update_count = 0
        self.update_mean_ms = 0.0
        self.update_max_ms = 0.0

        node.create_subscription(msg_type, topic, self._callback, 1)

    def parse(self, msg):
        """Read ``msg`` into ``self.data``. Called while holding the node lock."""
        raise NotImplementedError

    def rebase(self, data, origin):
        """Re-express ``data`` in the frame of ``origin``, this sensor's first
        measurement. Only needed when ``relative=True``."""
        raise NotImplementedError(
            f"{self.name}: relative=True but rebase() is not implemented")

    def _callback(self, msg):
        with self.node.lock:
            self.parse(msg)
            if self.relative:
                if self._origin is None:
                    self._origin = np.copy(self.data)
                self.rebase(self.data, self._origin)
            if not self._seeded and self.on_init is not None:
                self.on_init(self.data)
                self._seeded = True
            self.fresh = True

    def snapshot(self):
        """Return (fresh, copy of data) and clear the fresh flag. Call under lock."""
        fresh = self.fresh
        data = np.copy(self.data)
        self.fresh = False
        return fresh, data

    def residual(self, a, b):
        y = a - b
        if self.angle_idx is not None:
            y[self.angle_idx] = normalize_angle(y[self.angle_idx])
        return y

    def update(self, ekf, z, u):
        """Apply the sequential EKF update for this sensor and record timing."""
        t0 = time.perf_counter_ns()

        if self.uses_control:
            ekf.update(z=z, HJacobian=self.HJacobian, Hx=self.Hx, R=self.R,
                       args=u, hx_args=u, residual=self.residual)
        else:
            ekf.update(z=z, HJacobian=self.HJacobian, Hx=self.Hx, R=self.R,
                       residual=self.residual)

        duration_ms = (time.perf_counter_ns() - t0) / 1_000_000.0
        if duration_ms > self.update_max_ms:
            self.update_max_ms = duration_ms
        self.update_count += 1
        self.update_mean_ms += (duration_ms - self.update_mean_ms) / self.update_count


class ImuSensor(Sensor):
    """
    IMU sensor. Measurement: [yaw, yaw_rate, ax, ay].

    Applies an exponential moving average to the linear accelerations and can
    initialise the filter state from the first orientation reading.

    With ``relative=True`` the yaw is measured against the heading at the first
    message, which is what you want whenever the IMU's absolute heading has no
    common reference with the odometry sources: fusing two absolute yaws that
    differ by a constant offset leaves a residual the filter can never drive to
    zero. Rates and accelerations are already relative quantities.

    Args:
        on_init: optional callable invoked with the parsed measurement on the
                 first message, used to seed the filter state.
        alpha:   EMA weight for the newest acceleration sample.
    """

    def __init__(self, node, topic, R, Hx, HJacobian, on_init=None, alpha=0.2,
                 relative=False):
        super().__init__(node, 'imu', Imu, topic, 4, R, Hx, HJacobian,
                         uses_control=True, angle_idx=0, relative=relative,
                         on_init=on_init)
        self.alpha = alpha
        self.prev_ax = None
        self.prev_ay = None

    def rebase(self, data, origin):
        data[0] = normalize_angle(data[0] - origin[0])

    def parse(self, msg):
        new_ax = msg.linear_acceleration.x
        new_ay = msg.linear_acceleration.y

        if self.prev_ax is not None:
            self.data[2] = self.alpha * new_ax + (1 - self.alpha) * self.prev_ax
            self.data[3] = self.alpha * new_ay + (1 - self.alpha) * self.prev_ay
        else:  # first measurement
            self.data[2] = new_ax
            self.data[3] = new_ay

        self.prev_ax = self.data[2]
        self.prev_ay = self.data[3]

        self.data[1] = msg.angular_velocity.z

        q = [msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w]
        __, __, yaw = tft.euler_from_quaternion(q)
        self.data[0] = normalize_angle(yaw)


class OdomSensor(Sensor):
    """
    Full-state odometry sensor (e.g. VESC, VIO).
    Measurement: [x, y, yaw, vx, vy, yaw_rate].

    With ``relative=True`` the pose is re-anchored to the first message, so this
    source stops claiming an absolute origin. Use it when a second sensor already
    owns the absolute pose: two absolute pose sources anchored at different
    moments pull the filter apart.
    """

    def __init__(self, node, name, topic, R, Hx, HJacobian, angle_idx=2,
                 relative=False):
        super().__init__(node, name, Odometry, topic, 6, R, Hx, HJacobian,
                         uses_control=False, angle_idx=angle_idx,
                         relative=relative)

    def rebase(self, data, origin):
        # Rigid re-anchor of the pose into the first sample's frame. The
        # velocities are body-frame already, so they carry over untouched.
        dx = data[0] - origin[0]
        dy = data[1] - origin[1]
        c, s = np.cos(origin[2]), np.sin(origin[2])
        data[0] = c * dx + s * dy
        data[1] = -s * dx + c * dy
        data[2] = normalize_angle(data[2] - origin[2])

    def parse(self, msg):
        self.data[0] = msg.pose.pose.position.x
        self.data[1] = msg.pose.pose.position.y
        self.data[3] = msg.twist.twist.linear.x
        self.data[4] = msg.twist.twist.linear.y
        self.data[5] = msg.twist.twist.angular.z

        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        __, __, yaw = tft.euler_from_quaternion(q)
        self.data[2] = yaw
