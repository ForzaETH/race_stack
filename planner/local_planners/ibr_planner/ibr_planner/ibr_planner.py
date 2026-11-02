#!/usr/bin/env python3
"""ROS 2 game-theoretic (IBR) local planner -- Spica et al. 2018.

Drop-in replacement for the previous ibr_planner node: same topics, same message types,
same parameter names where they still apply. All of the optimisation lives in
inner_ocp.py; this file is ROS glue only.

    /centerline_waypoints      (f110_msgs/WpntArray)  -> track spline + half-widths
    /global_waypoints_scaled   (f110_msgs/WpntArray)  -> speed limits
    car_state/odom             (nav_msgs/Odometry)    -> ego state
    /<opp_ns>/car_state/odom   (nav_msgs/Odometry)    -> opponent state
    local_waypoints            (f110_msgs/WpntArray)  <- ego plan
    ibr/markers                (visualization_msgs/MarkerArray)

What changed relative to the previous node, and why the inner solve stops failing:

  1. The convexified subproblem carries L1-penalised SLACKS on the track and collision
     constraints, so it is feasible by construction. Previously, a measured state that
     already violated d_coll or the track width (which happens constantly: odometry noise,
     cars touching, a waypoint whose d_left is smaller than the car) made the QCLP
     infeasible, CLARABEL returned PRIMAL_INFEASIBLE, and the node published nothing.
  2. A trust region plus an accept/reject test on the TRUE (non-linearised) cost. The
     linearised track constraint is only valid near the iterate; without a trust region the
     first SCP step overshoots into the wall and the next linearisation is taken at a point
     outside the track.
  3. Track geometry is precomputed on a 1 cm arclength grid, so the projection used for the
     track constraint and for sigma (18) is a vectorised lookup rather than a per-waypoint
     nearest-neighbour search, and curvature comes from the spline instead of a finite
     difference of psi.
  4. The projection is windowed around the car's own arclength, so a plan never snaps onto
     a different part of the track that happens to run close by.
  5. The cvxpy problem is DPP and compiled ONCE; each SCP iteration only updates parameter
     values (~3 ms instead of ~35 ms).
  6. The duals handed to the opponent are clipped (mu_max). With slacks, a constraint held
     by its slack returns a dual equal to the slack penalty, which would otherwise swamp the
     progress term in (16) and drive the car straight at its opponent.

Parameters (ros2 param):
    rate_hz, dt, n_steps, opp_ns, speed_scale
    n_game_iterations   IBR sweeps per tick (each sweep = one best response per player)
    ibr_tolerance       stop the IBR sweeps once neither player's plan moves more than this
    n_sqp_iterations    max SCP iterations per best response
    inner_tolerance     ||theta^{m+1} - theta^m|| stopping threshold
    alpha, alpha_opp    aggressiveness gains of eq. (16); 0 = plain MPC vs a fixed opponent
    mu_max              clip on the collision duals passed between players
    r_col_ego, r_col_opp, track_margin
    trust_radius, trust_radius0, slack_weight, merit_weight
    viol_tolerance      how much constraint violation a published plan may carry [m]
    opp_timeout         after this long without opponent odometry, plan solo [s]
"""
import numpy as np
import json
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import (FloatingPointRange, ParameterDescriptor, ParameterType, SetParametersResult)

from f110_msgs.msg import Wpnt, WpntArray

try:        # installed as part of a package (setup.py entry point)
    from .inner_ocp import Params, RecedingHorizonIBR, Track, get_qclp, predict_opponent
except ImportError:                                  # run directly from the source folder
    from inner_ocp import Params, RecedingHorizonIBR, Track, get_qclp, predict_opponent


class IBRPlanner(Node):
    def __init__(self):
        super().__init__("ibr_planner")
        p = self.declare_parameter

        self.rate_hz = float(p("rate_hz", 10.0).value)
        self.dt = float(p("dt", 0.1).value)
        self.n_steps = int(p("n_steps", 15).value)
        self.opp_ns = str(p("opp_ns", "opp").value)
        self.speed_scale = float(p("speed_scale", 1.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=1.0, step=0.01)]
        )).value)
        self.opp_speed_scale = float(p("opp_speed_scale", 1.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=1.0, step=0.01)]
        )).value)



        self.speed_limit = float(p("speed_limit", 0.0).value)              # [m/s] 0 = use speed_scale
        self.rival_speed_limit = float(p("rival_speed_limit", 0.0).value)  # [m/s] 0 = use opp_speed_scale

        self.n_ibr = int(p("n_game_iterations", 4).value)
        self.ibr_tol = float(p("ibr_tolerance", 2e-2).value)
        self.alpha = (float(p("alpha", 1.0).value), float(p("alpha_opp", 1.0).value))
        self.opp_timeout = float(p("opp_timeout", 0.5).value)
        r_ego = float(p("r_col_ego", 0.29).value)
        r_opp = float(p("r_col_opp", 0.29).value)

        common = dict(
            N=self.n_steps, dt=self.dt, d_min=r_ego + r_opp,
            track_margin=float(p("track_margin", r_ego).value),
            max_iter=int(p("n_sqp_iterations", 6).value),
            tol=float(p("inner_tolerance", 1e-3).value),
            trust=float(p("trust_radius", 1.0).value),
            trust0=float(p("trust_radius0", 0.4).value),
            slack_w=float(p("slack_weight", 1e3).value),
            merit_w=float(p("merit_weight", 100.0).value),
            mu_max=float(p("mu_max", 2.0).value),
            proj_window=float(p("proj_window", 6.0).value),
            kappa_max=float(p("kappa_max", 1.2).value),

        )
        self.prm = Params(vmax=0.0, **common)          # vmax filled in from the global line
        self.prm_opp = Params(vmax=0.0, **common)
        self.viol_tol = float(p("viol_tolerance", 0.05).value)

        record_path = str(p("record_path", "").value)
        self.rec = open(record_path, "w") if record_path else None
        if self.rec:
            self.write_rec(dict(N=self.n_steps, dt=self.dt, d_min=self.prm.d_min,
                                margin=self.prm.track_margin,
                                proj_window=self.prm.proj_window, alpha=list(self.alpha),
                                n_ibr=self.n_ibr, kappa_max=self.prm.kappa_max))


        if self.rate_hz <= 0 or self.dt <= 0 or self.n_steps < 2 or self.n_ibr < 1:
            raise ValueError("Invalid rate, horizon or iteration count")
        if min(self.alpha) < 0:
            raise ValueError("alpha must be >= 0")

        self.state = np.zeros((2, 2))
        self.stamp = [None, None]
        self.t_meas = [0.0, 0.0]                        # odometry stamps of self.state [s]
        self.v_meas = [0.0, 0.0]                        # odometry speeds [m/s]
        self.track = None
        self.planner = None
        self.n_track_wpnts = 0
        self.gb_track = None
        self.n_gb_wpnts = 0

        # Canonicalise the convex subproblem now, not on the first planning tick.
        t0 = self.get_clock().now()
        get_qclp(self.n_steps)
        self.get_logger().info(
            f"QCLP compiled for N={self.n_steps} in "
            f"{(self.get_clock().now() - t0).nanoseconds / 1e6:.0f} ms")

        self.create_subscription(WpntArray, "/centerline_waypoints", self.centerline_cb, 10)
        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.gb_cb, 10)
        self.create_subscription(Odometry, "car_state/odom", self.ego_cb, 10)
        self.create_subscription(Odometry, f"/{self.opp_ns}/car_state/odom", self.opp_cb, 10)
        self.add_on_set_parameters_callback(self.param_cb)
        self.loc_pub = self.create_publisher(WpntArray, "local_waypoints", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "ibr/markers", 10)
        self.create_timer(1.0 / self.rate_hz, self.loop)

    # ------------------------------------------------------------------ callbacks
    def centerline_cb(self, msg: WpntArray):
        if len(msg.wpnts) < 4:
            self.get_logger().warn("centerline has fewer than four waypoints")
            return
        if len(msg.wpnts) == self.n_track_wpnts:
            return                                      # already built from this track
        self.n_track_wpnts = len(msg.wpnts)
        self.track = Track.from_wpnts(msg.wpnts, margin=self.prm.track_margin)
        self.planner = None                             # rebuilt on the next tick
        wl, wr = self.track.wlg, self.track.wrg
        self.get_logger().info(
            f"track: {self.n_track_wpnts} wpnts, L={self.track.L:.2f} m, usable half-width "
            f"L {wl.min():.2f}-{wl.max():.2f} m, R {wr.min():.2f}-{wr.max():.2f} m "
            f"(margin {self.prm.track_margin:.2f} m)")
        if min(wl.min(), wr.min()) <= 0.0:
            self.get_logger().warn(
                "the car half-width exceeds the track half-width somewhere; the planner "
                "will lean on its slacks there instead of reporting infeasible")

    def gb_cb(self, msg: WpntArray):
        if not msg.wpnts:
            return
        vmax_raw = max(float(w.vx_mps) for w in msg.wpnts)
        if vmax_raw <= 0.0:
            return
        self.prm.vmax = self.speed_limit or vmax_raw * self.speed_scale
        self.prm_opp.vmax = self.rival_speed_limit or vmax_raw * self.opp_speed_scale

        if len(msg.wpnts) != self.n_gb_wpnts:
            self.n_gb_wpnts = len(msg.wpnts)
            self.gb_track = Track.from_wpnts(msg.wpnts)   # only used for s, never for widths
            self.planner = None

    
    def ego_cb(self, msg: Odometry):
        self.state[0] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.stamp[0] = self.get_clock().now()
        self.t_meas[0] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.v_meas[0] = msg.twist.twist.linear.x


    def opp_cb(self, msg: Odometry):
        self.state[1] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.stamp[1] = self.get_clock().now()
        self.t_meas[1] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.v_meas[1] = msg.twist.twist.linear.x


    def param_cb(self, params):
        for param in params:
            if param.name == "speed_scale":
                self.speed_scale = param.value
            elif param.name == "opp_speed_scale":
                self.opp_speed_scale = param.value
        return SetParametersResult(successful=True)


    # ------------------------------------------------------------------ main loop
    def _opp_is_fresh(self):
        if self.stamp[1] is None:
            return False
        age = (self.get_clock().now() - self.stamp[1]).nanoseconds * 1e-9
        return age <= self.opp_timeout

    def write_rec(self, row):
        self.rec.write(json.dumps(row) + "\n")
        self.rec.flush()


    def loop(self):
        if self.track is None or self.stamp[0] is None or self.prm.vmax <= 0.0 or self.gb_track is None:
            return
        if self.planner is None:
            self.planner = RecedingHorizonIBR(
                self.track, self.prm, self.prm_opp, alpha=self.alpha,
                n_ibr=self.n_ibr, ibr_tol=self.ibr_tol, viol_tol=self.viol_tol,
                gb=self.gb_track)

        state = self.state.copy()
        game = self._opp_is_fresh()
        if game:
            out = self.planner.step(state[0], state[1])
            plan_opp = out["plan_opp"]
        else:
            # No opponent: plain OCP against a prediction parked far behind, which leaves
            # the collision constraint inactive. Keeps one code path for both cases.
            far = self.track.pos(self.track.project(state[0]) - 0.4 * self.track.L)
            q = predict_opponent(self.track, far, self.prm, v_opp=0.0)
            out = self.planner.step(state[0], far, q_opp=q)   # single OCP, no game
            plan_opp = q
            if self.stamp[1] is not None:
                self.get_logger().warn("opponent odometry is stale; planning solo",
                                       throttle_duration_sec=2.0)

        res = out["res"][0]
        R = out["ibr"]
        if self.rec:
            self.write_rec(dict(tick=self.planner.ticks, game=game, ok=bool(out["ok"]),
                                converged=bool(R.converged),
                                sweeps=len(R.history), solve_ms=out["time"] * 1e3,
                                vmax=[self.prm.vmax, self.prm_opp.vmax],
                                theta=[p.tolist() for p in R.p],
                                mu=[m.tolist() for m in R.mu],
                                s0g=[r.s0g for r in R.res],
                                t=list(self.t_meas), v=list(self.v_meas)))

        self.get_logger().info(
            f"IBR {out['time'] * 1e3:.1f} ms, {res.iters} SCP it, status={res.status}, "
            f"progress={out['progress']:.2f} m, viol track={res.max_track_viol:+.3f} "
            f"coll={res.max_coll_viol:+.3f}"
            f"IBR conv={R.converged} sweeps={len(R.history)}/{self.n_ibr}, "
        )

        if not out["ok"]:
            # The plan would be more infeasible than the state it started from: publish
            # nothing and let the supervisor fall back, exactly as before.
            self.get_logger().warn(
                f"rejecting plan: violation {out['worst_viol']:+.3f} m > allowed "
                f"{max(self.viol_tol, out['start_viol']):+.3f} m (status {res.status}, "
                f"{self.planner.rejects}/{self.planner.ticks} ticks rejected)",
                throttle_duration_sec=1.0)
            return
        if out["time"] > 1.0 / self.rate_hz:
            self.get_logger().warn(
                f"IBR took {out['time'] * 1e3:.0f} ms, longer than the "
                f"{1e3 / self.rate_hz:.0f} ms tick; lower n_game_iterations or "
                f"n_sqp_iterations", throttle_duration_sec=5.0)

        stamp = self.get_clock().now().to_msg()
        self.publish_plan(out["plan"], stamp, out["res"][0].s0g)
        self.publish_markers(out["plan"], plan_opp, stamp)

    # ------------------------------------------------------------------ output
    def publish_plan(self, plan, stamp, s0g, ds=0.1):
        """Resample the plan to `ds` spacing and publish it with every Wpnt field filled.

        The controllers pick their lookahead point by INDEX, assuming 0.1 m between
        waypoints (waypoint_at_distance_before_car), so the raw plan -- one point per dt,
        vmax*dt apart -- put the lookahead at the end of the horizon. plan[0] is the
        measured position and is kept, so the waypoint nearest the car is the first one.
        """
        seg = np.linalg.norm(np.diff(plan, axis=0), axis=1)
        v_seg = seg / self.dt                           # speed needed for each step
        keep = np.r_[True, seg > 1e-6]                  # np.interp needs increasing knots
        s = np.r_[0.0, np.cumsum(seg)][keep]
        v = np.r_[v_seg[0], v_seg][keep]                # knot k: speed of the step into it
        ss = np.arange(0.0, s[-1], ds)
        if len(ss) < 2:
            self.get_logger().warn("plan shorter than 2 waypoints; not publishing",
                                   throttle_duration_sec=1.0)
            return

        x, y = np.interp(ss, s, plan[keep, 0]), np.interp(ss, s, plan[keep, 1])
        vx = np.interp(ss, s, v)
        psi = np.arctan2(np.gradient(y), np.gradient(x))
        kappa = np.gradient(np.unwrap(psi)) / ds
        ax = vx * np.gradient(vx) / ds                  # a = v dv/ds

        # s/d are w.r.t. gb
        win = max(self.prm.proj_window, s[-1] + 1.0)
        _, _, _, _, _, lat, wl, wr = self.track.frenet(
            np.c_[x, y], self.track.project(plan[0]), win)
        m = self.track.margin                           # Track widths have the margin removed
        s_gb, _, _, _, _, d_gb, _, _ = self.gb_track.frenet(np.c_[x, y], s0g, win)

        out = WpntArray()
        out.header.stamp = stamp
        out.header.frame_id = "map"
        out.wpnts = [Wpnt(id=k, s_m=float(s_gb[k]), d_m=float(d_gb[k]),
                          x_m=float(x[k]), y_m=float(y[k]),
                          d_left=float(wl[k] + m - lat[k]), d_right=float(wr[k] + m + lat[k]),
                          psi_rad=float(psi[k]), kappa_radpm=float(kappa[k]),
                          vx_mps=float(vx[k]), ax_mps2=float(ax[k]))
                     for k in range(len(ss))]
        self.loc_pub.publish(out)

    def traj_marker(self, traj, marker_id, ns, rgb, stamp):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = stamp
        m.ns, m.id = ns, marker_id
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.scale.x = 0.05
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(seconds=2.0 / self.rate_hz).to_msg()
        m.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in traj]
        return m

    def publish_markers(self, plan, plan_opp, stamp):
        ma = MarkerArray()
        ma.markers.append(self.traj_marker(plan, 0, "ibr_ego", (0.0, 1.0, 0.0), stamp))
        ma.markers.append(self.traj_marker(plan_opp, 1, "ibr_opp", (1.0, 0.0, 0.0), stamp))
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = IBRPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
