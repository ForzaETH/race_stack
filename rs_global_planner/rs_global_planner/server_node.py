#!/usr/bin/env python3
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float64
from rs_msgs.msg import Sector, SectorArray, WpntArray
from visualization_msgs.msg import Marker, MarkerArray

from .plan import GlobalPlan

# latched: late subscribers still get the last message (publisher and subscriber must both use it)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class GlobalPlanServerNode(Node):
    """Serves the stored global plan of a map (global_waypoints.json), latched, for the rest of the stack."""

    def __init__(self):
        super().__init__('global_plan_server')

        map_dir = Path(self.declare_parameter(
            'map_path', rclpy.Parameter.Type.STRING,
            ParameterDescriptor(description='Map directory containing global_waypoints.json', read_only=True)).value)

        if not (map_dir / 'global_waypoints.json').exists():
            self.get_logger().fatal(f"No global plan in {map_dir}, run the global planner first (planner_launch.py)")
            raise SystemExit(1)
        
        # Load the computed plan
        plan = GlobalPlan.load(map_dir)

        # Publish the global waypoints (raceline)
        self.raceline_pub = self.create_publisher(WpntArray, '/global_plan/raceline', LATCHED)
        self.raceline_markers_pub = self.create_publisher(MarkerArray, '/global_plan/raceline/markers', LATCHED)
        self.track_length_pub = self.create_publisher(Float64, '/global_plan/raceline/track_length', LATCHED)
        
        # Publish the shortest path
        self.shortest_path_pub = self.create_publisher(WpntArray, '/global_plan/shortest_path', LATCHED)
        self.shortest_path_markers_pub = self.create_publisher(MarkerArray, '/global_plan/shortest_path/markers', LATCHED)
        
        # Publish the centerline
        self.centerline_pub = self.create_publisher(WpntArray, '/global_plan/centerline', LATCHED)
        self.centerline_markers_pub = self.create_publisher(MarkerArray, '/global_plan/centerline/markers', LATCHED)
        
        # Publish the track bounds
        self.track_bounds_markers_pub = self.create_publisher(MarkerArray, '/global_plan/track_bounds/markers', LATCHED)
        
        # Publish the sectors
        self.sectors_pub = self.create_publisher(SectorArray, '/global_plan/speed_sectors', LATCHED)
        
        self.publish_plan(plan)
        self.publish_sectors(plan.raceline, self.load_speed_sector_starts(map_dir, len(plan.raceline.wpnts)))

    def publish_plan(self, plan: GlobalPlan) -> None:
        self.track_length_pub.publish(Float64(data=_track_length(plan.raceline)))
        self.raceline_pub.publish(plan.raceline)
        self.shortest_path_pub.publish(plan.shortest_path)
        self.centerline_pub.publish(plan.centerline)

        # Publish the markers for the computed paths
        self.raceline_markers_pub.publish(MarkerArray(markers=[
            _line_strip(_xy(plan.raceline), 'raceline', RACELINE_COLOR, colors=_speed_colors(plan.raceline)),
            _direction_arrow(plan.raceline, 'raceline_direction', RACELINE_COLOR)]))
        self.shortest_path_markers_pub.publish(MarkerArray(markers=[
            _line_strip(_xy(plan.shortest_path), 'shortest_path', SHORTEST_PATH_COLOR,
                        colors=_speed_colors(plan.shortest_path)),
            _direction_arrow(plan.shortest_path, 'shortest_path_direction', SHORTEST_PATH_COLOR)]))
        self.centerline_markers_pub.publish(MarkerArray(markers=[
            _line_strip(_xy(plan.centerline), 'centerline', CENTERLINE_COLOR),
            _direction_arrow(plan.centerline, 'centerline_direction', CENTERLINE_COLOR)]))

        self.track_bounds_markers_pub.publish(MarkerArray(markers=[
            _line_strip(plan.bound_right, 'track_bound_right', TRACK_BOUNDS_COLOR),
            _line_strip(plan.bound_left, 'track_bound_left', TRACK_BOUNDS_COLOR)]))

    def publish_sectors(self, raceline: WpntArray, start_idxs: list[int]) -> None:
        s = [float(w.s_m) for w in raceline.wpnts]
        end_idxs = [i - 1 for i in start_idxs[1:]] + [len(s) - 1]
        end_s = [s[i] for i in start_idxs[1:]] + [_track_length(raceline)]
        self.sectors_pub.publish(SectorArray(sectors=[
            Sector(start_idx=start, end_idx=end, start_s=s[start], end_s=stop)
            for start, end, stop in zip(start_idxs, end_idxs, end_s)]))

    def load_speed_sector_starts(self, map_dir: Path, n_wpnts: int) -> list[int]:
        path = map_dir / 'speed_scaling.yaml'
        if not path.exists():
            self.get_logger().warn(f"No {path.name} in {map_dir}, publishing the whole lap as one sector")
            return [0]

        params = yaml.safe_load(path.read_text())['sector_tuner']['ros__parameters']
        starts = [int(params[f'Sector{i}']['start']) for i in range(params['n_sectors'])]

        # Sectors sliced on a different raceline (e.g. re-planned since) no longer fit it
        if not starts or starts[0] != 0 or starts != sorted(starts) or starts[-1] >= n_wpnts:
            self.get_logger().error(f"Sectors in {path.name} don't fit the raceline ({n_wpnts} waypoints), "
                                    "publishing the whole lap as one sector; slice the sectors again")
            return [0]
        return starts


RACELINE_COLOR = ColorRGBA(r=0.9, g=0.1, b=0.1, a=1.0)
SHORTEST_PATH_COLOR = ColorRGBA(r=0.1, g=0.4, b=1.0, a=1.0)
CENTERLINE_COLOR = ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0)
TRACK_BOUNDS_COLOR = ColorRGBA(r=0.1, g=0.8, b=0.1, a=1.0)


def _xy(wpnts: WpntArray) -> np.ndarray:
    return np.array([[w.x_m, w.y_m] for w in wpnts.wpnts])


def _track_length(wpnts: WpntArray) -> float:
    """Lap length in meters: s of the last waypoint plus the gap that closes the loop back to the first."""
    xy = _xy(wpnts)
    last_s = float(next(reversed([w.s_m for w in wpnts.wpnts])))
    return last_s + float(np.linalg.norm(xy[0] - xy[-1]))


# Anchors of matplotlib's inferno colormap from 0.2 to 1.0
SPEED_COLORMAP = np.array([[0.258, 0.039, 0.406],
                           [0.578, 0.148, 0.404],
                           [0.865, 0.317, 0.226],
                           [0.988, 0.645, 0.040],
                           [0.988, 0.998, 0.645]])


def _speed_colors(wpnts: WpntArray) -> list[ColorRGBA]:
    """One color per waypoint: slowest point of the line dark purple, fastest light yellow."""
    vx = np.array([w.vx_mps for w in wpnts.wpnts])
    t = (vx - vx.min()) / max(vx.max() - vx.min(), 1e-6)
    anchors = np.linspace(0.0, 1.0, len(SPEED_COLORMAP))
    rgb = np.column_stack([np.interp(t, anchors, SPEED_COLORMAP[:, i]) for i in range(3)])
    return [ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0) for r, g, b in rgb]


def _line_strip(points: np.ndarray, ns: str, color: ColorRGBA, width: float = 0.05, colors: list[ColorRGBA] | None = None) -> Marker:
    """Closed line through points [[x, y], ...] in the map frame; colors (one per point) override color."""
    marker = Marker(ns=ns, id=0, type=Marker.LINE_STRIP, action=Marker.ADD, color=color)
    marker.header.frame_id = 'map'
    marker.pose.orientation.w = 1.0
    marker.scale.x = width
    marker.points = [Point(x=float(x), y=float(y)) for x, y in np.vstack((points, points[:1]))]
    if colors is not None:
        marker.colors = colors + colors[:1]  # the closing point repeats the first one
    return marker


def _direction_arrow(wpnts: WpntArray, ns: str, color: ColorRGBA, length: float = 0.6) -> Marker:
    """Arrow at the first waypoint, pointing in the driving direction (its heading)."""
    start = next(iter(wpnts.wpnts))  # first waypoint; the generated field type allows a set, which can't be indexed
    marker = Marker(ns=ns, id=0, type=Marker.ARROW, action=Marker.ADD, color=color)
    marker.header.frame_id = 'map'
    marker.pose.orientation.w = 1.0
    marker.scale.x, marker.scale.y, marker.scale.z = 0.08, 0.2, 0.2  # shaft diameter, head diameter, head length
    marker.points = [Point(x=start.x_m, y=start.y_m),
                     Point(x=start.x_m + length * np.cos(start.psi_rad), y=start.y_m + length * np.sin(start.psi_rad))]
    return marker


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GlobalPlanServerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
