from pathlib import Path

import numpy as np
import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from rs_msgs.msg import WpntArray
from rs_msgs.srv import AddBoundary
from std_msgs.msg import ColorRGBA, Float64
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

SPEED_BOUNDARY_COLOR = ColorRGBA(r=0.2, g=0.55, b=1.0, a=1.0)
OVERTAKING_BOUNDARY_COLOR = ColorRGBA(r=0.9, g=0.15, b=0.15, a=1.0)


class SectorSet:
    """One set of sectors (speed or overtaking): its boundaries, slider position, markers, services and file."""

    def __init__(self, node: 'SectorSlicer', name: str, overtaking: bool):
        self.node = node
        self.overtaking = overtaking
        self.color = OVERTAKING_BOUNDARY_COLOR if overtaking else SPEED_BOUNDARY_COLOR
        self.boundaries = [0]
        self.cursor_idx = None  # waypoint index of the panel's slider position, once received
        self.saved = False

        prefix = f'/sector_slicer/{name}'
        
        self.markers_pub = node.create_publisher(MarkerArray, f'{prefix}/sectors', LATCHED)
        node.create_subscription(Float64, f'{prefix}/cursor_s', self.cursor_cb, 10)
        node.create_service(AddBoundary, f'{prefix}/add_boundary', self.add_boundary_cb)
        node.create_service(Trigger, f'{prefix}/save', self.save_cb)

    def reset(self) -> None:
        self.boundaries = [0]
        self.cursor_idx = None
        self.publish_boundaries()

    def cursor_cb(self, msg: Float64) -> None:
        if len(self.node.wpnts_s) == 0:
            return
        cursor_idx = self.node.nearest_wpnt(msg.data)  # as add_boundary
        if cursor_idx != self.cursor_idx:
            self.cursor_idx = cursor_idx
            self.publish_boundaries()

    def add_boundary_cb(self, request: AddBoundary.Request, response: AddBoundary.Response) -> AddBoundary.Response:
        wpnts_s = self.node.wpnts_s
        idx = self.node.nearest_wpnt(request.s_m)
        if idx in self.boundaries:
            response.success, response.message = False, f'There already is a boundary at s = {wpnts_s[idx]:.1f} m'
        else:
            self.boundaries = sorted(self.boundaries + [idx])
            self.publish_boundaries()
            response.success, response.message = True, f'Boundary added at s = {wpnts_s[idx]:.1f} m'
        return response

    def save_cb(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        path = self.write_yaml()
        self.saved = True
        response.success, response.message = True, f'{len(self.boundaries)} sectors saved to {path}'
        self.node.get_logger().info(response.message)
        self.node.on_saved()
        return response

    def publish_boundaries(self) -> None:
        """Publish a dot and label per boundary, and the slider position, as one marker array."""
        markers = [Marker(action=Marker.DELETEALL)]  # removed boundaries disappear
        if self.cursor_idx is not None:
            x, y = self.node.wpnts_xy[self.cursor_idx]
            cursor = Marker(ns='cursor', id=0, type=Marker.SPHERE, action=Marker.ADD, color=self.color)
            cursor.pose.position.x, cursor.pose.position.y = float(x), float(y)
            cursor.scale.x = cursor.scale.y = cursor.scale.z = 0.2
            markers.append(cursor)
        for i, idx in enumerate(self.boundaries):
            x, y = self.node.wpnts_xy[idx]
            dot = Marker(ns='boundary', id=i, type=Marker.SPHERE, action=Marker.ADD, color=self.color)
            dot.pose.position.x, dot.pose.position.y = float(x), float(y)
            dot.scale.x = dot.scale.y = dot.scale.z = 0.15
            label = Marker(ns='label', id=i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, color=self.color,
                           text=f'S{i}')
            label.pose.position.x, label.pose.position.y, label.pose.position.z = float(x), float(y), 0.4
            label.scale.z = 0.4
            markers += [dot, label]
        for marker in markers[1:]:
            marker.header.frame_id = 'map'
            marker.pose.orientation.w = 1.0
        self.markers_pub.publish(MarkerArray(markers=markers))

    def write_yaml(self) -> Path:
        """Write the sectors into the map folder, in the format sector_tuner/ot_interpolator read."""
        pnts = self.boundaries + [len(self.node.wpnts_s)]
        sectors = [{'start': pnts[i] if i == 0 else pnts[i] + 1, 'end': pnts[i + 1]} for i in range(len(pnts) - 1)]

        if self.overtaking:
            params = {'n_sectors': len(sectors), 'yeet_factor': 1.25, 'spline_len': 30, 'ot_sector_begin': 0.5}
            params.update({f'Overtaking_sector{i}': {**sector, 'ot_flag': False} for i, sector in enumerate(sectors)})
            node_name, file_name = 'ot_interpolator', 'ot_sectors.yaml'
        else:
            params = {'global_limit': 0.5, 'n_sectors': len(sectors)}
            params.update({f'Sector{i}': {**sector, 'scaling': 0.5} for i, sector in enumerate(sectors)})
            node_name, file_name = 'sector_tuner', 'speed_scaling.yaml'

        path = self.node.map_dir / file_name
        with open(path, 'w') as file:
            yaml.dump({node_name: {'ros__parameters': params}}, file, sort_keys=False)
        return path


class SectorSlicer(Node):
    """
    Splits the raceline into speed and overtaking sectors, controlled from the Lichtblick sector slicer panel.

    Exits once both sector sets are saved, so the launch shuts down with it.
    """

    def __init__(self):
        super().__init__('sector_slicer')

        # Obtain map directory
        self.map_dir = Path(str(self.declare_parameter('map_path', rclpy.Parameter.Type.STRING).value))

        self.wpnts_s = np.empty(0)
        self.wpnts_xy = np.empty((0, 2))

        # Completed once all sector sets are saved; the node then exits
        self.done = Future()

        self.sector_sets = [SectorSet(self, 'speed', overtaking=False), SectorSet(self, 'overtaking', overtaking=True)]
        self.create_subscription(WpntArray, '/global_waypoints', self.wpnts_cb, LATCHED)

        self.get_logger().info('Waiting for the raceline on /global_waypoints...')

    def wpnts_cb(self, msg: WpntArray) -> None:
        self.wpnts_s = np.array([w.s_m for w in msg.wpnts])
        self.wpnts_xy = np.array([[w.x_m, w.y_m] for w in msg.wpnts])
        for sector_set in self.sector_sets:
            sector_set.reset()
        self.get_logger().info(f'Raceline received ({len(self.wpnts_s)} waypoints), select the sectors in Lichtblick')

    def nearest_wpnt(self, s: float) -> int:
        """Index of the raceline waypoint closest to s (meters along the raceline)."""
        return int(np.argmin(np.abs(self.wpnts_s - s)))

    def on_saved(self) -> None:
        missing = [s for s in self.sector_sets if not s.saved]
        if missing:
            self.get_logger().info(f'{len(missing)} sector set(s) still to save')
        else:
            self.done.set_result(True)  # main() stops spinning once the save response is sent


def main():
    rclpy.init()
    try:
        node = SectorSlicer()
        rclpy.spin_until_future_complete(node, node.done)
    except (KeyboardInterrupt, ExternalShutdownException):
        return
    node.destroy_node()
    rclpy.shutdown()
