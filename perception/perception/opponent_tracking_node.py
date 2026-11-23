"""
ROS 2 wrapper around opponent_tracking: detections in, tracked obstacles out.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
from f110_msgs.msg import ObstacleArray, Obstacle, WpntArray
from stack_master.qos import LATCHED_QOS

from perception import opponent_tracking as ot
from perception import visualization as viz
from perception.opponent_tracking_parameters_codegen import opponent_tracking as parameters
from frenet_conversion.frenet_converter import FrenetConverter


class OpponentTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__("opponent_tracking")

        # Parameters: config/opponent_tracking_parameters.yaml
        self.params = parameters.ParamListener(self).get_params()

        # Opponent size from the rules (or an override): nominal length for the centre, maxima for the extent
        self.length_nom = float(np.mean(self.params.car_length_range))
        self.length_max = float(max(self.params.car_length_range))
        self.width_max = float(max(self.params.car_width_range))

        self.track_length: float | None = None        # from the raceline
        self.tracks: list[ot.Track] = []
        self.next_id = 0

        self.converter: FrenetConverter | None = None

        self.obstacles_pub = self.create_publisher(ObstacleArray, "/perception/obstacles", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/perception/obstacles/markers", 10)
        self.create_subscription(WpntArray, "/global_waypoints", self.waypoints_cb, LATCHED_QOS)
        self.create_subscription(ObstacleArray, "/perception/detections", self.detections_cb, 10)

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
        # What detection saw of each object (NaN = not seen): see obstacle_detection_node.publish_obstacles
        observations = [ot.Observation(o.s_start, o.s_end, o.d_left, o.d_right, o.s_center, o.d_center)
                        for o in msg.obstacles]
        self.tracks, self.next_id = ot.step_tracks(self.tracks, observations, t, self.track_length, self.next_id,
                                                   std_s=self.params.meas_std_s, std_d=self.params.meas_std_d,
                                                   coast_max=self.params.coast_max,
                                                   merge_length=self.length_max, merge_width=self.width_max)
        self.publish_tracks(t)

    def publish_tracks(self, t_meas: float) -> None:
        assert self.track_length is not None and self.converter is not None
        now = self.get_clock().now()
        dt = min(max(0.0, now.nanoseconds * 1e-9 - t_meas), 0.1)        # latency since the scan; capped

        obstacle_list, viz_tracks = [], []
        for tr in self.tracks:
            # Shape gate: only publish what could be a legal car (face >= ~12 cm, not wider than a car)
            if not tr.confirmed or not ot.plausible_shape(tr.shape, w_min=self.params.min_face_width,
                                                          w_max=self.width_max + 0.03,
                                                          n_min=self.params.min_face_samples):
                continue
            x, _ = ot.kf_predict(tr.x.copy(), tr.P.copy(), dt, self.track_length)
            s_rear, d_rear, vs, vd = (float(v) for v in x)

            # Heading relative to the raceline, from the track's own velocity (meaningless when slow: assume 0).
            # The centre lies half a car ahead of the rear ALONG THAT HEADING, not along the raceline.
            rel = float(np.arctan2(vd, vs)) if vs > self.params.min_speed_heading else 0.0
            s = (s_rear + self.length_nom / 2 * np.cos(rel)) % self.track_length     # centre of a nominal car
            d = d_rear + self.length_nom / 2 * np.sin(rel)

            # A car angled to the raceline covers more lateral space: widen each side by half its length * |sin|
            half_width = self.width_max / 2 + self.length_max / 2 * abs(np.sin(rel))

            o = Obstacle()
            o.id = tr.id
            o.s_center, o.d_center, o.vs, o.vd = s, d, vs, vd
            o.s_start = s_rear % self.track_length                       # observed rear
            o.s_end = (s_rear + self.length_max) % self.track_length     # conservative: longest legal car
            o.d_left, o.d_right = d + half_width, d - half_width
            o.size = max(self.length_max, self.width_max)
            o.is_static = not tr.dynamic
            o.is_visible = tr.t_last_hit == tr.t                         # detected in the latest scan
            obstacle_list.append(o)

            xy = np.asarray(self.converter.get_cartesian(s, d))
            viz_tracks.append((tr.id, xy, tr.dynamic, o.is_visible))

        msg = ObstacleArray()
        msg.header = Header(stamp=now.to_msg(), frame_id="map")
        msg.obstacles = obstacle_list
        self.obstacles_pub.publish(msg)
        self.markers_pub.publish(viz.track_markers(msg.header, viz_tracks))


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
