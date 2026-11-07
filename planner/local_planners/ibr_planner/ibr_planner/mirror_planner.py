#!/usr/bin/env python3
"""ROS 2 reactive mirroring defender.

The defender drives the gb raceline until the rival is close behind, then steers its
lateral offset (from the raceline) towards the rival's, so the rival never finds an open
lane. A deliberately degenerate baseline: it shows why racing needs rules (one move,
leave room) before an overtaking game becomes interesting.

    /global_waypoints_scaled   (f110_msgs/WpntArray)  -> raceline track, widths, speeds
    car_state/odom             (nav_msgs/Odometry)    -> own state (the defender)
    /<opp_ns>/car_state/odom   (nav_msgs/Odometry)    -> rival state (the attacker)
    local_waypoints            (f110_msgs/WpntArray)  <- plan, 0.1 m spacing
    mirror/markers             (visualization_msgs/MarkerArray)

Plan each tick, in the raceline Frenet frame: a smooth reference r(sigma) = d* kept inside
the usable band (track widths, and the steering limit on the inside of bends), plus a
quintic that takes the measured error (d - r, slope) to zero over Lt, where Lt grows with
the error so the lateral acceleration stays below a_lat. See plan_mirror.

Parameters (all re-read every tick, so `ros2 param set` works live):
    rate_hz, opp_ns, speed_scale, speed_limit
    mirror_delay  extra reaction delay on the rival's lateral offset [s]
    leave_room    overlap rule: once the rival overlaps, keep sep from it instead of mirroring
    engage_dist   mirror only while the rival is less than this far behind [m]
    wall_margin   removed from the raceline's d_left/d_right [m]
    t_lat, lt_min transition time [s] and minimum transition length [m]
    a_lat         lateral acceleration limit of the transition [m/s^2]
    kappa_car     path curvature the car can steer [1/m]; caps inside offsets in bends
    smooth        half-window of the reference smoothing [m]
    plan_len      published plan length [m]
    car_len, sep  footprint length [m]; lateral centre spacing that leaves room [m]
    opp_timeout   rival odometry older than this is ignored [s]
    record_path   optional jsonl log, one row per tick
"""
import json
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from f110_msgs.msg import Wpnt, WpntArray

try:        # installed as part of a package (setup.py entry point)
    from .inner_ocp import Track
except ImportError:                                  # run directly from the source folder
    from inner_ocp import Track


# --------------------------------------------------------------------------- pure planning
def quintic(d0, d1, d2, d_star, Lt):
    """Coefficients (highest power first, for np.polyval) of d(sigma) on [0, Lt] with
    d(0)=d0, d'(0)=d1, d''(0)=d2 and d(Lt)=d_star, d'(Lt)=0, d''(Lt)=0."""
    c0, c1, c2 = d0, d1, d2 / 2.0
    A = np.array([[Lt ** 3, Lt ** 4, Lt ** 5],
                  [3 * Lt ** 2, 4 * Lt ** 3, 5 * Lt ** 4],
                  [6 * Lt, 12 * Lt ** 2, 20 * Lt ** 3]])
    b = np.array([d_star - (c0 + c1 * Lt + c2 * Lt ** 2),
                  -(c1 + 2 * c2 * Lt),
                  -2 * c2])
    c3, c4, c5 = np.linalg.solve(A, b)
    return np.array([c5, c4, c3, c2, c1, c0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def target_offset(gap, d_def, d_att, d_att_seen, engage_dist, leave_room, car_len, sep):
    """d* from the rival's (delayed) offset. gap > 0: rival behind by gap [m]."""
    if not 0.0 < gap < engage_dist:
        return 0.0, "raceline"
    if leave_room and gap < car_len + 0.05 and abs(d_def - d_att) < sep:
        side = np.sign(d_def - d_att) or 1.0
        return d_att + side * sep, "leave_room"         # step aside, never squeeze
    return d_att_seen, "mirror"


def usable_band(track, s, kappa_car):
    """Left/right offset limits from the raceline: the track widths (margin already removed)
    and, on the inside of a bend, the offset at which the path curvature
    kappa / (1 - kappa d) reaches the car's limit kappa_car."""
    wl, wr = track.width(s)
    k = track.curvature(s)
    with np.errstate(divide="ignore"):
        inside = np.maximum(1.0 / np.abs(k) - 1.0 / kappa_car, 0.0)
    wl = np.where(k > 0, np.minimum(wl, inside), wl)
    wr = np.where(k < 0, np.minimum(wr, inside), wr)
    return wl, wr


def sliding(x, h, fn):
    """fn over a centred window of 2h+1 samples; x carries h extra samples on each side."""
    return fn(np.lib.stride_tricks.sliding_window_view(x, 2 * h + 1), axis=1)


def plan_mirror(track, s_def, d_def, yaw, v_def, d_star, t_lat, lt_min, plan_len,
                a_lat=3.0, kappa_car=1.2, smooth=0.8, ds=0.1):
    """Plan = smooth reference r(sigma) + a quintic that removes the car's current error.

    r is d* clipped to the usable band, made smooth without leaving it: clip to the band
    ERODED by +-smooth (its minimum over the window), then average over the same window.
    Every average then stays inside the band at its own point, and the width changes at
    apexes no longer put kinks into the path. The quintic takes the measured error
    e = d - r (value and slope) to zero over Lt, with Lt long enough that the lateral
    acceleration stays below a_lat (a quintic peaks at 5.77 |e| / Lt^2 in d'').

    Returns ss (unwrapped), d, Lt, r at Lt (the target the car settles on).
    """
    h = max(int(round(smooth / ds)), 1)
    plan_len = max(plan_len, 1.0)
    sig_x = np.arange(-2 * h, int(round(plan_len / ds)) + 2 * h) * ds   # 2h each side
    wl, wr = usable_band(track, s_def + sig_x, kappa_car)
    wl_e = sliding(wl, h, np.min)                                       # eroded: len - 2h
    wr_e = sliding(wr, h, np.min)
    r = sliding(np.clip(d_star, -wr_e, wl_e), h, np.mean)               # len - 4h = plan
    sig = sig_x[2 * h:len(sig_x) - 2 * h]

    psi_ref = math.atan2(*track.tangent(s_def)[::-1])
    kap = float(track.curvature(s_def))
    d1 = (1.0 - kap * d_def) * math.tan(np.clip(wrap(yaw - psi_ref), -1.2, 1.2))
    e0, e1 = d_def - r[0], d1 - (r[1] - r[0]) / ds
    dd_max = a_lat / max(v_def, 1.0) ** 2                               # d'' limit [1/m]
    Lt = max(lt_min, v_def * t_lat, math.sqrt(5.77 * abs(e0) / dd_max))
    Lt = min(Lt, sig[-1])
    coef = quintic(e0, e1, 0.0, 0.0, Lt)
    d = r + np.where(sig < Lt, np.polyval(coef, np.minimum(sig, Lt)), 0.0)
    i_t = min(int(round(Lt / ds)), len(r) - 1)
    return s_def + sig, d, Lt, float(r[i_t])


# --------------------------------------------------------------------------- ROS node
class MirrorPlanner(Node):
    PARAMS = dict(rate_hz=10.0, opp_ns="opp", speed_scale=1.0, speed_limit=0.0,
                  mirror_delay=0.0, leave_room=False, engage_dist=4.0, wall_margin=0.30,
                  t_lat=0.7, lt_min=1.0, plan_len=3.5, car_len=0.58, sep=0.41,
                  a_lat=3.0, kappa_car=1.2, smooth=0.8,
                  opp_timeout=0.5, record_path="")

    def __init__(self):
        super().__init__("mirror_planner")
        for name, default in self.PARAMS.items():
            self.declare_parameter(name, default)
        self.rate_hz = float(self.P("rate_hz"))
        self.opp_ns = str(self.P("opp_ns"))
        rec = str(self.P("record_path"))
        self.rec = open(rec, "w") if rec else None

        self.track = None
        self.margin = None
        self.gb_s = self.gb_v = None
        self.n_gb = 0
        self.ego = None                       # (x, y, yaw, v)
        self.opp = None                       # (x, y)
        self.opp_stamp = None
        self.s_hint = [None, None]
        self.hist = deque()                   # (t, d_att) for the reaction delay

        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.gb_cb, 10)
        self.create_subscription(Odometry, "car_state/odom", self.ego_cb, 10)
        self.create_subscription(Odometry, f"/{self.opp_ns}/car_state/odom", self.opp_cb, 10)
        self.loc_pub = self.create_publisher(WpntArray, "local_waypoints", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "mirror/markers", 10)
        self.create_timer(1.0 / self.rate_hz, self.loop)

    def P(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ callbacks
    def gb_cb(self, msg: WpntArray):
        margin = float(self.P("wall_margin"))
        if len(msg.wpnts) < 4 or (len(msg.wpnts) == self.n_gb and margin == self.margin):
            self.gb_v = np.array([w.vx_mps for w in msg.wpnts]) if self.gb_s is not None and \
                len(msg.wpnts) == len(self.gb_s) else self.gb_v      # speeds may be rescaled live
            return
        self.n_gb, self.margin = len(msg.wpnts), margin
        self.track = Track.from_wpnts(msg.wpnts, margin=margin)
        # arclength of each waypoint the way Track measures it (chords), for the speed lookup
        pts = np.array([[w.x_m, w.y_m] for w in msg.wpnts])
        v = np.array([w.vx_mps for w in msg.wpnts])
        if np.linalg.norm(pts[0] - pts[-1]) < 1e-7:
            pts, v = pts[:-1], v[:-1]
        self.gb_s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
        self.gb_v = v
        self.s_hint = [None, None]
        wl, wr = self.track.wlg, self.track.wrg
        self.get_logger().info(
            f"raceline: {len(pts)} wpnts, L={self.track.L:.2f} m, usable offset "
            f"left {wl.min():.2f}-{wl.max():.2f} m, right {wr.min():.2f}-{wr.max():.2f} m "
            f"(margin {margin:.2f} m)")

    def ego_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        self.ego = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                    msg.twist.twist.linear.x)

    def opp_cb(self, msg: Odometry):
        self.opp = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.opp_stamp = self.get_clock().now()

    # ------------------------------------------------------------------ main loop
    def _frenet(self, i, xy):
        s, _, _, _, _, d, _, _ = self.track.frenet(np.atleast_2d(xy), self.s_hint[i],
                                                   None if self.s_hint[i] is None else 6.0)
        self.s_hint[i] = float(s[0])
        return float(s[0]), float(d[0])

    def loop(self):
        if self.track is None or self.ego is None:
            return
        now = self.get_clock().now()
        t_now = now.nanoseconds * 1e-9
        x, y, yaw, v = self.ego
        s_def, d_def = self._frenet(0, (x, y))

        fresh = self.opp is not None and self.opp_stamp is not None and \
            (now - self.opp_stamp).nanoseconds * 1e-9 <= float(self.P("opp_timeout"))
        gap, d_att, d_seen = float("inf"), 0.0, 0.0
        if fresh:
            s_att, d_att = self._frenet(1, self.opp)
            gap = self.track.ds_along(s_att, s_def)            # > 0: rival behind
            self.hist.append((t_now, d_att))
            delay = float(self.P("mirror_delay"))
            while len(self.hist) > 1 and self.hist[1][0] <= t_now - delay:
                self.hist.popleft()                            # keep the newest sample old enough
            d_seen = self.hist[0][1]
        else:
            self.hist.clear()

        d_star, why = target_offset(gap, d_def, d_att, d_seen, float(self.P("engage_dist")),
                                    bool(self.P("leave_room")), float(self.P("car_len")),
                                    float(self.P("sep")))
        ss, d, Lt, d_star = plan_mirror(self.track, s_def, d_def, yaw, v, d_star,
                                        float(self.P("t_lat")), float(self.P("lt_min")),
                                        float(self.P("plan_len")), float(self.P("a_lat")),
                                        float(self.P("kappa_car")), float(self.P("smooth")))
        stamp = now.to_msg()
        xy = self.publish_plan(ss, d, stamp)
        self.publish_markers(xy, ss, d_star, Lt, stamp)

        if self.rec:
            self.rec.write(json.dumps(dict(t=t_now, s=s_def, d=d_def, v=v, gap=gap,
                                           d_att=d_att, d_seen=d_seen, d_star=d_star,
                                           mode=why)) + "\n")
            self.rec.flush()
        self.get_logger().info(
            f"{why:10s} gap={gap:5.2f} m  d={d_def:+.2f}  d_att={d_att:+.2f} "
            f"(seen {d_seen:+.2f})  d*={d_star:+.2f}  Lt={Lt:.2f} m",
            throttle_duration_sec=0.5)

    # ------------------------------------------------------------------ output
    def publish_plan(self, ss, d, stamp, ds=0.1):
        """Same fields and 0.1 m spacing as the IBR node: the controllers pick their
        lookahead point by index."""
        L = self.track.L
        xy = self.track.pos(ss) + d[:, None] * self.track.normal(ss)
        psi = np.arctan2(np.gradient(xy[:, 1]), np.gradient(xy[:, 0]))
        kappa = np.gradient(np.unwrap(psi)) / ds
        vx = np.interp(ss % L, self.gb_s, self.gb_v, period=L) * float(self.P("speed_scale"))
        lim = float(self.P("speed_limit"))
        if lim > 0.0:
            vx = np.minimum(vx, lim)
        ax = vx * np.gradient(vx) / ds
        wl, wr = self.track.width(ss)
        m = self.track.margin                           # Track widths have the margin removed
        out = WpntArray()
        out.header.stamp = stamp
        out.header.frame_id = "map"
        out.wpnts = [Wpnt(id=k, s_m=float(ss[k] % L), d_m=float(d[k]),
                          x_m=float(xy[k, 0]), y_m=float(xy[k, 1]),
                          d_left=float(wl[k] + m - d[k]), d_right=float(wr[k] + m + d[k]),
                          psi_rad=float(psi[k]), kappa_radpm=float(kappa[k]),
                          vx_mps=float(vx[k]), ax_mps2=float(ax[k]))
                     for k in range(len(ss))]
        self.loc_pub.publish(out)
        return xy

    def marker(self, mid, ns, mtype, rgb, stamp, pts, scale):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = stamp
        m.ns, m.id = ns, mid
        m.type, m.action = mtype, Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(seconds=2.0 / self.rate_hz).to_msg()
        m.points = [Point(x=float(px), y=float(py), z=0.0) for px, py in pts]
        return m

    def publish_markers(self, xy, ss, d_star, Lt, stamp):
        s_t = ss[0] + Lt
        tgt = self.track.pos(s_t) + d_star * self.track.normal(s_t)
        ma = MarkerArray()
        ma.markers.append(self.marker(0, "mirror_plan", Marker.LINE_STRIP, (0.0, 0.6, 1.0),
                                      stamp, xy, 0.05))
        ma.markers.append(self.marker(1, "mirror_target", Marker.SPHERE_LIST, (1.0, 0.5, 0.0),
                                      stamp, [tgt], 0.15))
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = MirrorPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.rec:
            node.rec.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
