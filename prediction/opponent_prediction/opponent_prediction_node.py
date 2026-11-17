"""ROS 2 node: learns the opponent's trajectory with a Gaussian process and publishes it."""

import time
import numpy as np
import torch
import rclpy
import threading
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from f110_msgs.msg import ObstacleArray, OppWpnt, OpponentTrajectory, WpntArray
from stack_master.qos import LATCHED_QOS

from opponent_prediction.gp import OpponentPrediction, OpponentTrajectoryGP
from opponent_prediction.parameters_codegen import opponent_prediction as parameters


class OpponentPredictionNode(Node):
    def __init__(self) -> None:
        super().__init__("opponent_prediction")

        # Parameters: defined (with defaults, descriptions and bounds) in config/opponent_prediction_parameters.yaml
        self.params = parameters.ParamListener(self).get_params()

        self.track_length: float | None = None
        self.s_ref: np.ndarray | None = None
        self.v_ref: np.ndarray | None = None
        self.xy_ref: np.ndarray | None = None      # raceline points in the map frame [m]
        self.normal_ref: np.ndarray | None = None  # unit normals pointing left (+d) [-]

        # Our current position
        self.ego_s: float | None = None

        # The opponent is the closest dynamic obstacle ahead of us (within a window)
        self.lap = 0
        self.last_s: float | None = None
        self.samples: list[tuple[float, float, float, int]] = []  # (s, d, vs, lap)

        # Fitting runs in a background thread, periodically
        self.n_added = 0
        torch.set_num_threads(self.params.fit_threads)
        self.n_added_at_fit = 0
        self.fit_thread: threading.Thread | None = None
        self.prediction: OpponentPrediction | None = None
        self.create_timer(self.params.fit_period, self.fit_cb)

        # Consistency check: the opponent is off the prediction after n_off_max detections in a row beyond z_max std
        self.n_off = 0
        self.on_trajectory = False
        self.traj_msg: OpponentTrajectory | None = None

        self.traj_pub = self.create_publisher(OpponentTrajectory, "/opponent_trajectory", LATCHED_QOS)
        self.marker_pub = self.create_publisher(MarkerArray, "/opponent_trajectory/markers", LATCHED_QOS)

        self.create_subscription(WpntArray, "/global_waypoints", self.waypoints_cb, LATCHED_QOS)
        self.create_subscription(Odometry, "/car_state/frenet/odom", self.ego_cb, 10)
        self.create_subscription(ObstacleArray, "/perception/obstacles", self.obstacles_cb, 10)

        self.get_logger().info("Waiting for /global_waypoints ...")

    def waypoints_cb(self, msg: WpntArray) -> None:
        wpnts = list(msg.wpnts)
        self.track_length = wpnts[-1].s_m  # last point's s is the track length
        self.s_ref = np.array([w.s_m for w in wpnts[:-1]])
        self.v_ref = np.array([w.vx_mps for w in wpnts[:-1]])
        self.xy_ref = np.array([[w.x_m, w.y_m] for w in wpnts[:-1]])
        psi = np.array([w.psi_rad for w in wpnts[:-1]])
        self.normal_ref = np.column_stack([-np.sin(psi), np.cos(psi)])

        self.get_logger().info(f"Track length {self.track_length:.1f} m, {len(self.s_ref)} waypoints")

    def ego_cb(self, msg: Odometry) -> None:
        self.ego_s = msg.pose.pose.position.x

    def obstacles_cb(self, msg: ObstacleArray) -> None:
        if self.track_length is None or self.ego_s is None:
            return
        L = self.track_length

        # Obtain the gap to the dynamic obstacle (within the gap window)
        gaps = [((o.s_center - self.ego_s) % L, o) for o in msg.obstacles if not o.is_static and o.is_visible]
        gaps = [(g, o) for g, o in gaps if self.params.min_gap < g < self.params.max_gap]
        if not gaps:
            return

        # The closest opponent is what we need
        opp = min(gaps, key=lambda x: x[0])[1]
        s = opp.s_center % L
        self.check_on_trajectory(s, opp.d_center)

        if self.last_s is not None:
            ds = s - self.last_s
            if ds < -L / 2:                     # crossed the finish line
                self.lap += 1
                self.samples = [x for x in self.samples if x[3] >= self.lap - self.params.laps_kept]
            elif ds > L / 2:                    # jittered back over it
                self.lap -= 1
            elif abs(ds) < self.params.ds_min:         # too close, drop it
                return
        self.last_s = s
        self.samples.append((s, opp.d_center, opp.vs, self.lap))
        self.n_added += 1

    def check_on_trajectory(self, s: float, d: float) -> None:
        """Compare a detection with the prediction; republish the trajectory when the verdict changes."""
        p = self.prediction
        if p is None:
            return
        i = min(int(np.searchsorted(p.s, s)), len(p.s) - 1)
        z = abs(d - p.d_mean[i]) / np.hypot(p.d_std[i], self.params.d_noise_std)
        self.n_off = self.n_off + 1 if z > self.params.z_max else 0
        on = self.n_off < self.params.n_off_max
        if on == self.on_trajectory:
            return
        self.on_trajectory = on
        self.get_logger().info(f"Opponent {'on' if on else 'off'} the predicted trajectory")
        if self.traj_msg is not None:
            self.traj_msg.opp_is_on_trajectory = on
            self.traj_pub.publish(self.traj_msg)

    def fit_cb(self) -> None:
        if self.fit_thread is not None and self.fit_thread.is_alive():
            return  # previous fit still running
        if self.n_added - self.n_added_at_fit < self.params.min_new:
            return
        if self.track_length is None or self.s_ref is None or self.v_ref is None:
            return
        self.n_added_at_fit = self.n_added
        data = np.array(self.samples)  # snapshot: the callbacks keep appending meanwhile
        gp = OpponentTrajectoryGP(self.track_length, self.s_ref, self.v_ref)
        self.fit_thread = threading.Thread(target=self.fit, args=(gp, data), daemon=True)
        self.fit_thread.start()

    def fit(self, gp: OpponentTrajectoryGP, data: np.ndarray) -> None:
        s, d, vs, lap = data.T
        ok = vs > 0  # the speed model is in log space
        t0 = time.perf_counter()
        try:
            gp.fit(s[ok], d[ok], vs[ok], (lap[ok] - lap.min()).astype(int),
                   self.params.d_noise_std**2, self.params.vs_noise_std**2)
            self.prediction = gp.predict(gp.s_ref, self.params.n_std)
        except Exception as e:  # a failed fit must not kill the node
            self.get_logger().error(f"GP fit failed: {e}")
            return

        self.traj_msg = self.to_msg(self.prediction)
        self.traj_pub.publish(self.traj_msg)
        self.get_logger().info(
            f"GP fit: {ok.sum()} samples, {len(np.unique(lap))} laps, {time.perf_counter() - t0:.1f} s")
        self.marker_pub.publish(self.to_markers(self.prediction))

    def to_msg(self, p: OpponentPrediction) -> OpponentTrajectory:
        xy = self.to_xy(p.d_mean)
        vs_std = (p.vs_upper - p.vs_lower) / (2 * self.params.n_std)  # band half-width in std
        msg = OpponentTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.lap_count = self.lap
        msg.opp_is_on_trajectory = self.on_trajectory
        msg.oppwpnts = [
            OppWpnt(id=i, s_m=float(p.s[i]), d_m=float(p.d_mean[i]), x_m=float(xy[i, 0]), y_m=float(xy[i, 1]),
                    proj_vs_mps=float(p.vs_median[i]), vd_mps=0.0,
                    d_var=float(p.d_std[i] ** 2), vs_var=float(vs_std[i] ** 2))
            for i in range(len(p.s))
        ]
        return msg

    def to_xy(self, d: np.ndarray) -> np.ndarray:
        """Map-frame points at lateral offset d from our raceline, one per waypoint."""
        assert self.xy_ref is not None and self.normal_ref is not None
        return self.xy_ref + d[:, None] * self.normal_ref

    def to_markers(self, p: OpponentPrediction) -> MarkerArray:
        """Predicted line coloured by speed (red slow, green fast) and the n_std lateral band in grey."""
        f = (p.vs_median - p.vs_median.min()) / max(np.ptp(p.vs_median), 1e-6)
        lines = [
            ("mean", p.d_mean, 0.06, [ColorRGBA(r=1.0 - float(x), g=float(x), b=0.0, a=1.0) for x in f]),
            ("band_left", p.d_mean + self.params.n_std * p.d_std, 0.02, None),
            ("band_right", p.d_mean - self.params.n_std * p.d_std, 0.02, None),
        ]
        stamp = self.get_clock().now().to_msg()
        markers: list[Marker] = []
        for ns, d, width, colors in lines:
            m = Marker()
            m.header.stamp, m.header.frame_id = stamp, "map"
            m.ns, m.id, m.type, m.action = ns, 0, Marker.LINE_STRIP, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = width
            m.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.8)
            xy = self.to_xy(d)
            m.points = [Point(x=float(x), y=float(y)) for x, y in xy] + [Point(x=float(xy[0, 0]), y=float(xy[0, 1]))]
            if colors is not None:
                m.colors = colors + colors[:1]  # one per point, closing the loop
            markers.append(m)
        return MarkerArray(markers=markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpponentPredictionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RCLError:
            pass


if __name__ == "__main__":
    main()
