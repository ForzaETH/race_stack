import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from rs_msgs.msg import WpntArray
from visualization_msgs.msg import Marker, MarkerArray

from .plan import GlobalPlan

# latched: late subscribers still get the last message (publisher and subscriber must both use it)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class GlobalPlanPublisher:
    """Latched publishers for a GlobalPlan: its waypoints and their markers. Publishing once is enough."""

    def __init__(self, node: Node):
        self.raceline_pub = node.create_publisher(WpntArray, '/global_waypoints', LATCHED)
        self.raceline_markers_pub = node.create_publisher(MarkerArray, '/global_waypoints/markers', LATCHED)
        self.shortest_path_pub = node.create_publisher(WpntArray, '/global_waypoints/shortest_path', LATCHED)
        self.shortest_path_markers_pub = node.create_publisher(MarkerArray, '/global_waypoints/shortest_path/markers', LATCHED)
        self.centerline_pub = node.create_publisher(WpntArray, '/centerline_waypoints', LATCHED)
        self.centerline_markers_pub = node.create_publisher(MarkerArray, '/centerline_waypoints/markers', LATCHED)
        self.track_bounds_markers_pub = node.create_publisher(MarkerArray, '/trackbounds/markers', LATCHED)

    def publish(self, plan: GlobalPlan) -> None:
        self.raceline_pub.publish(plan.raceline)
        self.shortest_path_pub.publish(plan.shortest_path)
        self.centerline_pub.publish(plan.centerline)

        # Raceline and shortest path colored by their speed profile; each line with an arrow for the driving direction
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

        bound_right, bound_left = _track_bounds(plan.centerline)
        self.track_bounds_markers_pub.publish(MarkerArray(markers=[
            _line_strip(bound_right, 'track_bound_right', TRACK_BOUNDS_COLOR),
            _line_strip(bound_left, 'track_bound_left', TRACK_BOUNDS_COLOR)]))


RACELINE_COLOR = ColorRGBA(r=0.9, g=0.1, b=0.1, a=1.0)
SHORTEST_PATH_COLOR = ColorRGBA(r=0.1, g=0.4, b=1.0, a=1.0)
CENTERLINE_COLOR = ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0)
TRACK_BOUNDS_COLOR = ColorRGBA(r=0.1, g=0.8, b=0.1, a=1.0)


def _xy(wpnts: WpntArray) -> np.ndarray:
    return np.array([[w.x_m, w.y_m] for w in wpnts.wpnts])


def _track_bounds(wpnts: WpntArray) -> tuple[np.ndarray, np.ndarray]:
    """Right and left track bound, rebuilt from waypoint positions and their distances to the bounds."""
    xy = _xy(wpnts)
    psi = np.array([w.psi_rad for w in wpnts.wpnts])
    left_normal = np.column_stack((-np.sin(psi), np.cos(psi)))
    d_right = np.array([w.d_right for w in wpnts.wpnts])[:, None]
    d_left = np.array([w.d_left for w in wpnts.wpnts])[:, None]
    return xy - d_right * left_normal, xy + d_left * left_normal


# Anchors of matplotlib's inferno colormap from 0.2 to 1.0 (the darkest part is skipped, it vanishes on dark backgrounds)
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


def _line_strip(points: np.ndarray, ns: str, color: ColorRGBA, width: float = 0.05,
                colors: list[ColorRGBA] | None = None) -> Marker:
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
