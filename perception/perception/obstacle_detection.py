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


def yaw_from_quat(x, y, z, w) -> float:
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _se2_compose(a, b) -> np.ndarray:
    """Chain two SE(2) transforms (T1 @ T2)"""
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


def segment_scan(idx, xy, r, angle_inc, lambda_=np.radians(10), sigma=0.03, max_missing=3) -> np.ndarray:
    """
    Segment id for each valid beam: a new segment starts wherever the gap to the previous beam exceeds
    a distance.
    """
    if len(idx) == 0:
        return np.zeros(0, int)

    # Difference (angle and distance) between two consecutive (valid) beams
    dphi = np.diff(idx) * angle_inc
    gap = np.linalg.norm(np.diff(xy, axis=0), axis=1)

    # Find the largest allowed distance where two points still count as the same surface
    with np.errstate(divide="ignore", invalid="ignore"):
        # Law of sines (each side divided by the opposite angle gives the same value)
        d_max = r[:-1] * np.sin(dphi) / np.sin(lambda_ - dphi) + 3 * sigma

    # Break down the beams into segments with the adaptive thresholds
    brk = (dphi >= lambda_) | (gap > d_max)

    # More than max_missing dropped beams in a row: always a new segment (d_max explodes as dphi -> lambda_)
    brk |= np.diff(idx) > max_missing + 1
    return np.concatenate((np.zeros(1, int), np.cumsum(brk)))


def accept_segments(labels, point_ok) -> np.ndarray:
    """True only for points whose whole segment is ok."""
    if len(labels) == 0:
        return np.zeros(0, bool)  # no points, nothing to keep

    # There are labels.max() + labels
    bad = np.zeros(labels.max() + 1, bool)

    # Segment is bad if any point is bad. Strictness reduces false positives, at the cost of some detection rate
    # Relaxing this constraint can significantly increase FPs.
    np.logical_or.at(bad, labels, ~point_ok)
    return ~bad[labels]


def group_segments(xy_map, labels, keep, max_gap=0.25) -> list[np.ndarray]:
    """Combine kept segments closer than max_gap to each other; e.g. car side and rear face."""
    groups = []
    for seg in np.unique(labels[keep]):
        members = np.flatnonzero(keep & (labels == seg))
        merged = False
        for i, group in enumerate(groups):

            # Smallest disance between any point of this segment and any point in the group
            gap = np.linalg.norm(xy_map[members][:, None, :] - xy_map[group][None, :, :], axis=2).min()

            # Group segments closer than max_gap
            if gap < max_gap:
                groups[i] = np.concatenate((group, members))
                merged = True
                break
        if not merged:
            groups.append(members)
    return groups


def plausible_group(xy_map, r, members, angle_inc, face_width=0.30, frac=0.3, min_pts=4, max_extent=0.75) -> bool:
    """True if a group could be a car-sized obstacle."""
    rng = np.median(r[members])

    # Beams a car-wide face would cover at this range; require a fraction of them
    expected = face_width / (rng * angle_inc)
    if len(members) < max(min_pts, frac * expected):
        return False

    # Bounding-box diagonal must fit a car
    pts = xy_map[members]
    extent = np.linalg.norm(pts.max(axis=0) - pts.min(axis=0))
    return bool(extent <= max_extent)


def candidate_mask(xy_map, r, grid: Grid, inside, wall_dist, m0=0.10, k_range=0.005) -> np.ndarray:
    """True for points that are on the track and are clear of the walls."""
    row, col, ok = grid.world_to_grid(xy_map)
    keep = np.zeros(len(xy_map), bool)

    # Keep points inside the track but not too close to the walls
    rr, cc = row[ok], col[ok]
    keep[ok] = inside[rr, cc] & (wall_dist[rr, cc] > m0 + k_range * r[ok])
    return keep


def closeness_heading(pts, prior, search=np.radians(30), step=np.radians(1), d0=0.01, prior_weight=0.03) -> float:
    """
    Heading of the box that best fits the points (closeness criterion: points near the box edges score high).
    Searches within +-search of the prior (the raceline heading at the obstacle), with a soft penalty for
    straying from it.
    """
    
    best_score, best_heading = -np.inf, prior
    
    for theta in np.arange(prior - search, prior + search + 1e-9, step):
        # Points in the box axes
        u = pts @ np.array([np.cos(theta), np.sin(theta)])
        v = pts @ np.array([np.cos(theta + np.pi / 2), np.sin(theta + np.pi / 2)])
        
        # Distance of each point to the nearest edge of the bounding rectangle
        du = np.minimum(u - u.min(), u.max() - u)
        dv = np.minimum(v - v.min(), v.max() - v)
        
        # Each point is weighted by the edge its closest to
        d = np.maximum(np.minimum(du, dv), d0)
        score = np.mean(1.0 / d) * d0 - prior_weight * ((theta - prior) / search) ** 2
        
        if score > best_score:
            best_score, best_heading = score, theta

    return best_heading


def end_is_real(ranges, beam, direction, r_end, tol=0.10, max_look=3) -> bool:
    """
    Whether the end of a segment is a real edge of the object, or cut off by something in front of it.
    Looks at the next valid beam beyond the end, before the first beam, and after the last.
    """
    n = len(ranges)
    for k in range(1, max_look + 1):
        j = beam + direction * k
        
        # End of the scan: the object may continue outside the field of view -> not a real edge
        if j < 0 or j >= n:
            return False
        
        rj = ranges[j]
        
        # Skip dropped beams
        if not np.isfinite(rj) or rj < 0.02:
            continue
        
        # Background farther away => real edge. Nearer => occluded
        return rj > r_end - tol

    return True

def main_segment(labels, members) -> np.ndarray:
    """The largest contiguous segment of a group."""
    segments, counts = np.unique(labels[members], return_counts=True)
    return members[labels[members] == segments[np.argmax(counts)]]

def observe_face(pts, beams, r, ranges, sensor, theta, face_depth=0.05, edge_tol=0.03):
    """
    What of the object's outline is actually seen, in its own axes (heading theta).
    Returns four map points (2,), NaN where not observed:
        rear:  centre of the rear face
        front: centre of the front face
        left:  a point on the left edge (only if it is a real edge)
        right: a point on the right edge (only if it is a real edge)
    """
    fwd = np.array([np.cos(theta), np.sin(theta)])
    lft = np.array([-np.sin(theta), np.cos(theta)])

    # Points and sensor in the object's axes: u forward, v left
    u, v = pts @ fwd, pts @ lft
    su, sv = sensor @ fwd, sensor @ lft
    umin, umax, vmin, vmax = u.min(), u.max(), v.min(), v.max()
    
     # Is each angular end of the segment (first / last beam) a real edge, or cut off by an occluder?
    i0, i1 = np.argmin(beams), np.argmax(beams)
    real0 = end_is_real(ranges, beams[i0], -1, r[i0])
    real1 = end_is_real(ranges, beams[i1], +1, r[i1])

    def trusted(values, extreme):
        """An extent can be used unless it is set by an occluded end."""
        for i, real in ((i0, real0), (i1, real1)):
            if not real and abs(values[i] - extreme) < edge_tol:
                return False
        return True

    u_rear = u_front = v_left = v_right = np.nan
    
    # The sensor is behind / ahead of all points: we look at the rear / front face
    behind, ahead = su < umin, su > umax
    if behind or ahead:
        # Points of the face we look at: the outermost face_depth along u
        face = (u < umin + face_depth) if behind else (u > umax - face_depth)
        if behind:
            u_rear = np.median(u[face])
        else:
            u_front = np.median(u[face])

        # Lateral edges from the face points only: a side seen under a small heading error can't widen it
        if trusted(v, vmin):
            v_right = v[face].min()
        if trusted(v, vmax):
            v_left = v[face].max()

    # Otherwise we look at a side: its ends give rear / front, the side itself gives one lateral edge
    else:
        if trusted(u, umin):
            u_rear = umin
        if trusted(u, umax):
            u_front = umax
        if sv > vmax:
            v_left = vmax
        elif sv < vmin:
            v_right = vmin
            
    # Back to map points. Lateral position for rear / front: face centre if both edges seen
    v_c = 0.5 * (v_left + v_right) if np.isfinite(v_left) and np.isfinite(v_right) else np.median(v)
    u_ref = u_rear if np.isfinite(u_rear) else (u_front if np.isfinite(u_front) else np.median(u))

    def point(uu, vv):
        return uu * fwd + vv * lft          # NaN in, NaN out

    return point(u_rear, v_c), point(u_front, v_c), point(u_ref, v_left), point(u_ref, v_right)

def frenet_box(s, d, track_length) -> tuple[float, float, float, float]:
    """Bounding box of points in Frenet coordinates. Returns (s_mid, d_mid, length along s, width along d)."""
    
    ref = s[0]
    ds = (s - ref + track_length / 2) % track_length - track_length / 2
    lo, hi = ds.min(), ds.max()
    
    s_mid = (ref + (lo + hi) / 2) % track_length
    d_mid = (d.min() + d.max()) / 2
    return float(s_mid), float(d_mid), float(hi - lo), float(d.max() - d.min())
