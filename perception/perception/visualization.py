"""
RViz messages for the perception nodes. Display only: nothing here feeds back into detection or tracking.
"""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
          for i, n in enumerate(("x", "y", "z", "segment", "dt"))]


def points_cloud(header: Header, xy, segment, dt) -> PointCloud2:
    """Points (N, 2) as a PointCloud2, with their segment label and beam time as extra fields."""
    pts = np.column_stack((xy, np.zeros(len(xy)), segment, dt)).astype(np.float32)
    return point_cloud2.create_cloud(header, FIELDS, pts)


def observation_markers(header: Header, observations) -> MarkerArray:
    """Observed faces [(centroid, rear, front, left, right)] as spheres: rear red, front blue, edges green,
    centroid grey. Unseen (NaN) points are skipped. Clears the previous scan's markers first."""
    colours = ((0.5, 0.5, 0.5), (1.0, 0.1, 0.1), (0.1, 0.3, 1.0), (0.1, 0.9, 0.1), (0.1, 0.9, 0.1))
    markers = [Marker(action=Marker.DELETEALL)]
    for i, obs in enumerate(observations):
        for j, (p, (cr, cg, cb)) in enumerate(zip(obs, colours)):
            if not np.all(np.isfinite(p)):
                continue
            m = Marker()
            m.header = header
            m.ns, m.id, m.type, m.action = "detections", 5 * i + j, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(p[0]), float(p[1]), 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.06
            m.color.r, m.color.g, m.color.b, m.color.a = cr, cg, cb, 0.9
            markers.append(m)
    return MarkerArray(markers=markers)


def live_wall_marker(header: Header, walls, offsets, full_scale: float = 0.15) -> Marker:
    """Learned wall points (N, 2), coloured by offset: red = moved into the track, blue = moved out."""
    m = Marker()
    m.header = header
    m.ns, m.id, m.type, m.action = "live_walls", 0, Marker.POINTS, Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = 0.04
    points, colors = [], []
    for (x, y), off in zip(walls, offsets):
        points.append(Point(x=float(x), y=float(y), z=0.0))
        k = float(np.clip(off / full_scale, -1.0, 1.0))
        colors.append(ColorRGBA(r=max(k, 0.0), g=0.2, b=max(-k, 0.0), a=1.0))
    m.points, m.colors = points, colors
    return m


def track_markers(header: Header, tracks) -> MarkerArray:
    """
    Tracks [(id, xy, dynamic, visible)] as cylinders: green = dynamic, grey = static, faint = coasting.
    Clears the previous tracks first.
    """
    markers = [Marker(action=Marker.DELETEALL)]
    for track_id, xy, dynamic, visible in tracks:
        m = Marker()
        m.header = header
        m.ns, m.id, m.type, m.action = "tracks", int(track_id), Marker.CYLINDER, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(xy[0]), float(xy[1]), 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.10      # small: marks the centre, the car itself is visible in the scan
        m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b = (0.1, 0.8, 0.1) if dynamic else (0.5, 0.5, 0.5)
        m.color.a = 0.9 if visible else 0.3
        markers.append(m)
    return MarkerArray(markers=markers)
