"""
Obstacle detection from one LiDAR scan. Pure Python, no ROS.
Static map, built once from the planner's track bounds.

Per scan:
    - beam_geometry     valid beams in laser frame (with individual beam times)
    - segment_scan      split the scan at range jumps
    - beams_to_map      map frame, laser pose interpolated per beam
    - candidate_mask    point is on track clear of the walls
    - accept_segments   keep a segment only if all if its points pass


False positives are the worst failures (as detecting an obstacle when there is none can
cause a behavioural error). Hence this code can handle walls which are deviated.
"""

from dataclasses import dataclass
import numpy as np
import cv2


@dataclass
class Grid:
    """The map is discretized into a grid for obstacle detection."""
    origin_xy: np.ndarray    # map origin in map frame [m]
    resolution: float        # size of a cell [m]
    width: int
    height: int

    def world_to_grid(self, xy) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Convert world coordinates to discrete grid coordinates."""
        # Grid origin is bottom left
        x, y = xy[:, 0], xy[:, 1]
        x0, y0 = self.origin_xy

        row = np.floor((y - y0) / self.resolution).astype(int)
        col = np.floor((x - x0) / self.resolution).astype(int)
        ok = (row >= 0) & (row < self.height) & (col >= 0) & (col < self.width)

        return row, col, ok


def grid_from_bounds(right, left, res, pad) -> Grid:
    """Construct Grid covering all the wall points from the d_left and d_right of the waypoints."""
    pts = np.vstack((right, left))

    # Construct the tightest rectangle encompassing all wall points
    bottom_left_corner_wall_pt = pts.min(axis=0)
    top_right_wall_pt = pts.max(axis=0)

    # Pad the rectangle
    lo = bottom_left_corner_wall_pt - pad
    hi = top_right_wall_pt + pad

    # Count the amount of cells
    width, height = np.ceil((hi - lo) / res).astype(int)

    return Grid(origin_xy=lo, resolution=res, width=int(width), height=int(height))


def _to_pixels(grid: Grid, xy) -> np.ndarray:
    """World coordinates to pixel coordinates (row, col)"""
    px = (xy - grid.origin_xy) / grid.resolution - 0.5  # -0.5 as the center of a cell is the middle
    return np.round(px).astype(np.int32).reshape(-1, 1, 2)


def _area(xy) -> float:
    """Area of a polygon using the shoelace formula."""
    x, y = xy[:, 0], xy[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def track_mask(grid: Grid, left, right) -> np.ndarray:
    """Construct an binary occupancy grid of the driveable track."""

    # Determine the outer and inner boundaries by comparing the areas of both polygons
    outer, inner = (right, left) if _area(right) > _area(left) else (left, right)
    mask = np.zeros((grid.height, grid.width), np.uint8)
    cv2.fillPoly(mask, [_to_pixels(grid, outer)], (1,))
    cv2.fillPoly(mask, [_to_pixels(grid, inner)], (0,))
    return mask.astype(bool)


def wall_distance(grid: Grid, right, left) -> np.ndarray:
    """
    Computes distance [m] to the nearest wall. Perceived obstacles too close to the expected wall are
    discarded. This is critical to avoid the car seeing ghosts which can cause unpredictable behaviour
    for the overtaking planners.
    """
    walls = np.zeros((grid.height, grid.width), np.uint8)
    cv2.polylines(walls, [_to_pixels(grid, right), _to_pixels(grid, left)], isClosed=True, color=(1,), thickness=1)
    free = (walls == 0).astype(np.uint8)
    return cv2.distanceTransform(free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * grid.resolution


def beam_geometry(ranges, angle_min, angle_inc, time_inc, range_min, range_max):
    """Valid beams of one scan, as laser frame points."""

    ranges = np.asarray(ranges, dtype=float)

    # Keep only valid beams
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    idx = np.flatnonzero(valid)
    r = ranges[idx]

    # Beam end points in the laser frame
    a = angle_min + idx * angle_inc
    xy = np.column_stack((r * np.cos(a), r * np.sin(a)))

    # Time of each beam relative to header.stamp (= FIRST beam, ROS LaserScan convention)
    dt = idx * time_inc
    return idx, xy, r, dt


def _yaw_from_quat(x, y, z, w) -> float:
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _se2_compose(a, b) -> np.ndarray:
    """Chain two SE(2) transforms."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    return np.stack((a[..., 0] + c * b[..., 0] - s * b[..., 1],
                     a[..., 1] + s * b[..., 0] + c * b[..., 1],
                     a[..., 2] + b[..., 2]), axis=-1)


def _se2_interp(p0, p1, alpha) -> np.ndarray:
    """Linear interpolation between poses."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    d[2] = np.arctan2(np.sin(d[2]), np.cos(d[2]))
    return p0 + np.asarray(alpha, float)[:, None] * d


def _se2_apply(pose, xy) -> np.ndarray:
    """Move points from a child frame into its parent frame."""
    pose = np.asarray(pose, float)
    c, s = np.cos(pose[..., 2]), np.sin(pose[..., 2])
    return np.column_stack((pose[..., 0] + c * xy[:, 0] - s * xy[:, 1],
                            pose[..., 1] + s * xy[:, 0] + c * xy[:, 1]))


def beams_to_map(xy, dt, scan_duration, map_odom, odom_laser_start, odom_laser_end) -> np.ndarray:
    """
    Laser frame points to the map frame, each with its own beam time.
    """

    # For each beam, how far through the sweep it fired
    alpha = dt / scan_duration if scan_duration > 0 else np.zeros_like(dt)

    # One laser pose per beam (per-beam motion correction)
    odom_laser = _se2_interp(odom_laser_start, odom_laser_end, alpha)

    # Chain map→odom and odom→laser to obtain map→laser per beam. Then apply the transfrom on each beam.
    return _se2_apply(_se2_compose(map_odom, odom_laser), xy)


def segment_scan(idx, xy, r, angle_inc, lambda_=np.radians(10), sigma=0.03) -> np.ndarray:
    """
    Segment id for each valid beam: a new segment starts wherever the gap to the previous beam exceeds
    a distance.
    """
    if len(idx) == 0:
        return np.zeros(0, int)

    # Angle between two consecutive (valid) beams
    dphi = np.diff(idx) * angle_inc
    gap = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        d_max = r[:-1] * np.sin(dphi) / np.sin(lambda_ - dphi) + 3 * sigma
    brk = (dphi >= lambda_) | (gap > d_max)
    return np.concatenate((np.zeros(1, int), np.cumsum(brk)))


def accept_segments(labels, point_ok) -> np.ndarray:
    """True only for points whose whole segment is ok."""
    bad = np.zeros(labels.max() + 1, bool) if len(labels) else np.zeros(0, bool)
    np.logical_or.at(bad, labels, ~point_ok)   # segment is bad if any point is bad
    return ~bad[labels]


def candidate_mask(xy_map, r, grid: Grid, inside, wall_dist, m0=0.10, k_yaw=0.005) -> np.ndarray:
    """True for points that are on the track and are clear of the walls."""
    row, col, ok = grid.world_to_grid(xy_map)
    keep = np.zeros(len(xy_map), bool)

    # Keep points inside the track but not too close to the walls
    rr, cc = row[ok], col[ok]
    keep[ok] = inside[rr, cc] & (wall_dist[rr, cc] > m0 + k_yaw * r[ok])
    return keep
