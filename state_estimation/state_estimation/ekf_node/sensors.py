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
from scipy.stats import chi2
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
        gate_groups:   optional list of index lists. Each group of measurement rows
                       is chi^2-gated independently on its Mahalanobis distance;
                       only accepted rows are fused. None disables gating.
        gate_prob:     chi^2 quantile used as acceptance threshold (e.g. 0.99).
        gate_max_reject: after this many consecutive rejections of a group, the
                       group is force-accepted once to re-anchor a drifted filter.
    """

    def __init__(self, node, name, msg_type, topic, dim, R, Hx, HJacobian,
                 uses_control=False, angle_idx=None, relative=False, on_init=None,
                 gate_groups=None, gate_prob=0.99, gate_max_reject=50):
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

        # Innovation gating
        self.gate_groups = [list(g) for g in gate_groups] if gate_groups else None
        self.gate_thresholds = ([chi2.ppf(gate_prob, df=len(g)) for g in self.gate_groups]
                                if self.gate_groups else None)
        self.gate_max_reject = gate_max_reject
        self.gate_consecutive = [0] * len(self.gate_groups) if self.gate_groups else None
        self.reject_count = 0
        self.force_accept_count = 0

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

    def _gate(self, ekf, z, u):
        """Return the sorted list of measurement rows that pass the chi^2 gate."""
        args = (u,) if self.uses_control else ()
        H = np.atleast_2d(self.HJacobian(ekf.x, *args))
        y = self.residual(np.asarray(z, dtype=float), np.asarray(self.Hx(ekf.x, *args), dtype=float))
        S = H @ ekf.P @ H.T + self.R

        rows = []
        for i, (group, thr) in enumerate(zip(self.gate_groups, self.gate_thresholds)):
            S_g = S[np.ix_(group, group)]
            y_g = y[group]
            d2 = float(y_g @ np.linalg.solve(S_g, y_g))
            if d2 <= thr:
                self.gate_consecutive[i] = 0
                rows.extend(group)
            elif self.gate_consecutive[i] >= self.gate_max_reject:
                self.gate_consecutive[i] = 0
                self.force_accept_count += 1
                self.node.get_logger().warn(
                    f"[EKF Node] {self.name}: rows {group} rejected {self.gate_max_reject} "
                    f"times in a row (d2={d2:.1f}), force-accepting to re-anchor",
                    throttle_duration_sec=1.0)
                rows.extend(group)
            else:
                self.gate_consecutive[i] += 1
                self.reject_count += 1
        return sorted(rows)

    def update(self, ekf, z, u):
        """Apply the sequential EKF update for this sensor and record timing."""
        t0 = time.perf_counter_ns()

        if self.gate_groups is None:
            rows = None
        else:
            rows = self._gate(ekf, z, u)
            if len(rows) == 0:
                return

        if rows is None or len(rows) == len(z):
            Hx, HJacobian, R, z_used, residual = self.Hx, self.HJacobian, self.R, z, self.residual
        else:
            idx = np.array(rows)
            Hx = lambda x, *a: np.asarray(self.Hx(x, *a)).flatten()[idx]
            HJacobian = lambda x, *a: np.atleast_2d(self.HJacobian(x, *a))[idx, :]
            R = self.R[np.ix_(idx, idx)]
            z_used = np.asarray(z)[idx]
            ang = rows.index(self.angle_idx) if self.angle_idx in rows else None

            def residual(a, b, ang=ang):
                y = a - b
                if ang is not None:
                    y[ang] = normalize_angle(y[ang])
                return y

        ekf.dim_z = len(z_used)
        if self.uses_control:
            ekf.update(z=z_used, HJacobian=HJacobian, Hx=Hx, R=R,
                       args=u, hx_args=u, residual=residual)
        else:
            ekf.update(z=z_used, HJacobian=HJacobian, Hx=Hx, R=R,
                       residual=residual)

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

    Additionally maintains ``ax_lt``: the longitudinal acceleration, corrected
    with the filter's current accelerometer bias and low-passed with time
    constant ``tau_lt`` at the IMU rate. It is used as a model input (load
    transfer only), separately from the accelerometer measurement.

    Args:
        on_init: optional callable invoked with the parsed measurement on the
                 first message, used to seed the filter state.
        alpha:   EMA weight for the newest acceleration sample in the
                 measurement (1.0 disables the EMA).
        bias_fn: optional callable returning the current accelerometer x bias
                 estimate; defaults to zero bias.
        tau_lt:  time constant [s] of the ax_lt low-pass.
    """

    def __init__(self, node, topic, R, Hx, HJacobian, on_init=None, alpha=0.2,
                 relative=False, bias_fn=None, tau_lt=0.03, **gating):
        super().__init__(node, 'imu', Imu, topic, 4, R, Hx, HJacobian,
                         uses_control=True, angle_idx=0, relative=relative,
                         on_init=on_init, **gating)
        self.alpha = alpha
        self.prev_ax = None
        self.prev_ay = None
        self.bias_fn = bias_fn
        self.tau_lt = tau_lt
        self.ax_lt = 0.0
        self._last_stamp = None

    def rebase(self, data, origin):
        data[0] = normalize_angle(data[0] - origin[0])

    def _update_ax_lt(self, msg, ax_raw):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        b_ax = self.bias_fn() if self.bias_fn is not None else 0.0
        a = ax_raw - b_ax
        # Gravity is not removed here: assumes a flat track / negligible pitch.
        if self._last_stamp is None:
            self.ax_lt = a
        else:
            dt = stamp - self._last_stamp
            if 0.0 < dt < 0.5:
                self.ax_lt += dt / (self.tau_lt + dt) * (a - self.ax_lt)
        self._last_stamp = stamp

    def parse(self, msg):
        new_ax = msg.linear_acceleration.x
        new_ay = msg.linear_acceleration.y
        self._update_ax_lt(msg, new_ax)

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
                 relative=False, **gating):
        super().__init__(node, name, Odometry, topic, 6, R, Hx, HJacobian,
                         uses_control=False, angle_idx=angle_idx,
                         relative=relative, **gating)

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


class VescSpeedInput:
    """
    Wheel-speed source used as a model INPUT (not a measurement update).

    Reads the longitudinal speed from the VESC odometry, which vesc_to_odom
    already computes as (ERPM - speed_to_erpm_offset) / speed_to_erpm_gain,
    i.e. the nominal wheel surface speed v_w. The effective-radius error is
    estimated by the model's s_w state.
    """

    def __init__(self, node, topic):
        self.node = node
        self.name = 'vesc_speed'
        self.v_w = 0.0
        self.received = False
        node.create_subscription(Odometry, topic, self._callback, 1)

    def _callback(self, msg):
        with self.node.lock:
            self.v_w = msg.twist.twist.linear.x
            self.received = True

    def snapshot(self):
        """Return the latest wheel speed. Call under lock."""
        return self.v_w
