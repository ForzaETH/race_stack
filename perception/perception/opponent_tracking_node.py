"""
ROS 2 wrapper around opponent_tracking: detections in, tracked obstacles out.
"""

import copy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from f110_msgs.msg import ObstacleArray, Obstacle, WpntArray
from stack_master.qos import LATCHED_QOS

from perception import opponent_tracking as ot
from frenet_conversion.frenet_converter import FrenetConverter


class OpponentTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__("opponent_tracking")

        self.std_s = self.declare_parameter("meas_std_s", 0.05).get_parameter_value().double_value   # [m]
        self.std_d = self.declare_parameter("meas_std_d", 0.03).get_parameter_value().double_value   # [m]
        self.coast_max = self.declare_parameter("coast_max", 0.7).get_parameter_value().double_value  # [s]
        self.opp_length = self.declare_parameter("opp_length", 0.55).get_parameter_value().double_value
        self.opp_width = self.declare_parameter("opp_width", 0.30).get_parameter_value().double_value
        self.R = np.diag([self.std_s ** 2, self.std_d ** 2])

        self.track_length: float | None = None        # from the raceline
        self.tracks: list[ot.Track] = []
        self.next_id = 0

        self.converter: FrenetConverter | None = None

        self.obstacles_pub = self.create_publisher(ObstacleArray, "/perception/v2/obstacles", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/perception/v2/obstacles/markers", 10)
        self.create_subscription(WpntArray, "/global_waypoints", self.waypoints_cb, LATCHED_QOS)
        self.create_subscription(ObstacleArray, "/perception/v2/detections", self.detections_cb, 10)

    def waypoints_cb(self, msg: WpntArray) -> None:
        x = np.array([w.x_m for w in msg.wpnts])
        y = np.array([w.y_m for w in msg.wpnts])
        psi = np.array([w.psi_rad for w in msg.wpnts])
        self.converter = FrenetConverter(x, y, psi)
        self.track_length = float(np.array([w.s_m for w in msg.wpnts])[-1])

    def detections_cb(self, msg: ObstacleArray) -> None:
        """One detection message per scan: update all tracks, then publish."""
        if self.track_length is None:
            return
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9          # measurement time of the scan
        detections = [[o.s_center, o.d_center] for o in msg.obstacles]
        self.tracks, self.next_id = ot.step_tracks(self.tracks, detections, t, self.R, self.track_length,
                                                   self.next_id, coast_max=self.coast_max)
        self.publish_tracks(t)

    def publish_tracks(self, t_meas: float) -> None:
        assert self.track_length is not None and self.converter is not None
        now = self.get_clock().now()
        dt = min(max(0.0, now.nanoseconds * 1e-9 - t_meas), 0.1)        # latency since the scan; capped

        obstacle_list, marker_list = [], [Marker(action=Marker.DELETEALL)]
        for tr in self.tracks:
            if not tr.confirmed:
                continue
            x, _ = ot.kf_predict(tr.x.copy(), tr.P.copy(), dt, self.track_length)
            s, d, vs, vd = (float(v) for v in x)

            o = Obstacle()
            o.id = tr.id
            o.s_center, o.d_center, o.vs, o.vd = s, d, vs, vd
            o.s_start = (s - self.opp_length / 2) % self.track_length
            o.s_end = (s + self.opp_length / 2) % self.track_length
            o.d_left, o.d_right = d + self.opp_width / 2, d - self.opp_width / 2
            o.size = max(self.opp_length, self.opp_width)
            o.is_static = not tr.dynamic
            o.is_visible = tr.t_last_hit == tr.t                         # detected in the latest scan
            obstacle_list.append(o)

            xy = np.asarray(self.converter.get_cartesian(s, d))
            m = Marker()
            m.header = Header(stamp=now.to_msg(), frame_id="map")
            m.ns, m.id, m.type, m.action = "tracks", tr.id, Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(xy[0]), float(xy[1]), 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.35
            m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b = (0.1, 0.8, 0.1) if tr.dynamic else (0.5, 0.5, 0.5)   # green / grey
            m.color.a = 0.9 if o.is_visible else 0.3                                               # faint = coasting
            marker_list.append(m)

        msg = ObstacleArray()
        msg.header = Header(stamp=now.to_msg(), frame_id="map")
        msg.obstacles = obstacle_list
        self.obstacles_pub.publish(msg)
        self.markers_pub.publish(MarkerArray(markers=marker_list))


def main(args=None):
    rclpy.init(args=args)
    node = OpponentTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
