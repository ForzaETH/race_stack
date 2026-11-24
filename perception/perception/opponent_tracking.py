"""
Opponent tracking in Frenet coordinates. Pure Python, no ROS.

State x = [s, d, vs, vd]: position along / across the raceline [m] and their speeds [m/s]
Constant velocity Kalman filter.
"""

from dataclasses import dataclass, field
import numpy as np


MIN_FACE = 0.12  # Minimum mandated size of the LiDAR box


@dataclass
class Observation:
    """What detection saw of one object, in Frenet. NaN where not seen."""
    s_rear: float        # rear face
    s_front: float       # front face
    d_left: float        # left edge
    d_right: float       # right edge
    s_c: float           # centre of the bounding box of all points (always set): fallback position
    d_c: float
    length: float = np.nan     # bounding box of all the object's points, along s [m]
    width: float = np.nan      # ... along d [m]
    heading: float = np.nan    # heading in the map frame from the L-shape fit [rad]


@dataclass
class Shape:
    """Learned size of what we see of one object."""
    width: float = np.nan      # lateral extent of a face seen from behind / ahead [m]
    widths: list = field(default_factory=list)


@dataclass
class Track:
    """One tracked object."""
    id: int
    x: np.ndarray           # [s_rear, d, vs, vd]: rear-centre position and speeds
    P: np.ndarray
    t: float
    t_last_hit: float       # last time a detection was assigned
    hits: list = field(default_factory=list)
    confirmed: bool = False
    dynamic: bool = False
    history: list = field(default_factory=list)   # recent (t, s, d) at each detection: dynamic label, stationarity
    shape: Shape = field(default_factory=Shape)    # learned size of what we see of it
    heading: float = np.nan                        # smoothed heading in the map frame [rad], NaN until seen
    static_confirmed: bool = False                  # still for >= t_static: a static object
    extent: tuple | None = None                     # union of observed boxes while still, ABSOLUTE (s_lo, s_hi,
                                                    # d_lo, d_hi) [m]; s_hi = s_lo + length, may exceed the track length


def new_track(track_id, z, t, R, v0_std=3.0) -> Track:
    """Start a track at detection z = [s, d]; speed unknown (+-v0_std m/s)."""
    x = np.array([z[0], z[1], 0.0, 0.0])
    P = np.diag([R[0, 0], R[1, 1], v0_std ** 2, v0_std ** 2])
    return Track(id=track_id, x=x, P=P, t=t, t_last_hit=t, hits=[True])


def wrap_s(s, track_length):
    return s % track_length


def wrap_ds(ds, track_length):
    """Difference in s into [-track_length/2, track_length/2] so the start/finish line doesn't jump."""
    return (ds + track_length / 2) % track_length - track_length / 2


def update_shape(shape: Shape, obs: Observation, keep=200) -> None:
    """Fold one observation into the learned shape (in place)."""

    # Both lateral edges seen: only happens looking at the rear or front face -> its width
    if np.isfinite(obs.d_left) and np.isfinite(obs.d_right):
        shape.widths = (shape.widths + [obs.d_left - obs.d_right])[-keep:]
        shape.width = float(np.median(shape.widths))


def plausible_shape(shape: Shape, w_min=0.10, w_max=0.356, n_min=10) -> bool:
    """Plausible shape for a car: at least a face >= 12 cm (LiDAR box), and max width of 32.6 cm. """
    return len(shape.widths) >= n_min and w_min <= shape.width <= w_max


def kf_predict(x, P, dt, track_length, sigma_as=7.0, sigma_ad=3.0):
    """
    Move the state dt seconds forward at constant velocity.
    sigma_as, sigma_ad: acceleration noise along / across the track [m/s^2]
    """

    # s_new = s + vs * dt, d_new = d + vd * dt, vs_new = vs, vd_new = vd
    F = np.array([[1, 0, dt, 0],
                  [0, 1, 0, dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]], dtype=float)

    # Random acceleration a over dt moves position by a * (dt^2 / 2) and speed by a * (dt)
    qp = (dt ** 2 / 2) ** 2             # position variance
    qv = dt ** 2                        # speed variance
    qpv = (dt ** 2 / 2) * dt            # position-speed covariance

    Q = np.zeros((4, 4))
    # Along the track: s (index 0) and vs (index 2)
    Q[0, 0] = qp * sigma_as ** 2
    Q[0, 2] = Q[2, 0] = qpv * sigma_as ** 2
    Q[2, 2] = qv * sigma_as ** 2
    # Across the track d (index 1) and vd (index 3)
    Q[1, 1] = qp * sigma_ad ** 2
    Q[1, 3] = Q[3, 1] = qpv * sigma_ad ** 2
    Q[3, 3] = qv * sigma_ad ** 2

    # Get new prediction
    x = F @ x
    x[0] = wrap_s(x[0], track_length)
    P = F @ P @ F.T + Q
    return x, P


def kf_update(x, P, z, R, track_length):
    """
    Correct the state with a measured position z = [s, d] (measurement noise R, 2x2).
    Returns the new state and its uncertainty.
    """
    H = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0]], dtype=float)

    # Measurement minus prediction
    y = z - H @ x
    y[0] = wrap_ds(y[0], track_length)
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)

    x = x + K @ y
    x[0] = wrap_s(x[0], track_length)
    I_KH = np.eye(4) - K @ H
    P = I_KH @ P @ I_KH.T + K @ R @ K.T
    return x, P


def nis_of(x, P, z, R, track_length) -> float:
    """How well detection z fits a track's predicted position."""
    y = np.asarray(z, float) - x[:2]
    y[0] = wrap_ds(y[0], track_length)
    S = P[:2, :2] + R
    return float(y @ np.linalg.inv(S) @ y)


def update_motion_class(tr: Track, track_length, v_up=0.8, v_down=0.2, window=0.5, min_span=0.45,
                        max_lateral=0.5) -> None:
    """
    Static / dynamic from how far the track moved over the last `window` seconds.
    Dynamic only if it moved ALONG the track faster than v_up, with little sideways motion
    (|dd| < max_lateral * |ds|): a static structure that only seems to move, because more of it
    becomes visible as we approach, shifts inconsistently or sideways. Static again below v_down.
    """
    h = [x for x in tr.history if tr.t - x[0] <= window]    # history is filled in update_static_evidence

    span = h[-1][0] - h[0][0]
    if span < min_span:
        return                                     # not enough history yet: keep the current label

    ds = wrap_ds(h[-1][1] - h[0][1], track_length)
    dd = h[-1][2] - h[0][2]
    along = ds / span
    if not tr.dynamic and along > v_up and abs(dd) < max_lateral * abs(ds):
        tr.dynamic = True
    elif tr.dynamic and abs(along) < v_down:
        tr.dynamic = False


def anchor(obs: Observation, shape: Shape, track_length, std_s=0.05, std_d=0.03,
           std_unknown_s=0.30, std_unknown_d=0.10):
    """
    Measurement z = [s_rear, d] of a track's rear-centre from one observation, and its noise R (2x2).
    Missing parts are filled in with the learned shape; where the shape is unknown, the noise is raised
    so the filter barely moves on that axis.
    """
    # Along the track: rear seen directly, else the front with an unknown length (high noise: barely moves s)
    if np.isfinite(obs.s_rear):
        s, ss = obs.s_rear, std_s
    elif np.isfinite(obs.s_front):
        s, ss = obs.s_front - MIN_FACE, std_unknown_s
    else:
        s, ss = obs.s_c, std_unknown_s

    # Across the track: both edges -> their middle, one edge -> half the learned width inside it
    half = shape.width / 2 if np.isfinite(shape.width) else MIN_FACE / 2
    sd_one = 1.5 * std_d if np.isfinite(shape.width) else std_unknown_d
    if np.isfinite(obs.d_left) and np.isfinite(obs.d_right):
        d, sdd = 0.5 * (obs.d_left + obs.d_right), std_d
    elif np.isfinite(obs.d_left):
        d, sdd = obs.d_left - half, sd_one
    elif np.isfinite(obs.d_right):
        d, sdd = obs.d_right + half, sd_one
    else:
        d, sdd = obs.d_c, std_unknown_d

    z = np.array([s % track_length, d])
    R = np.diag([ss ** 2, sdd ** 2])
    return z, R

def update_heading(tr: Track, heading, alpha) -> None:
    """
    Smooth the track's map-frame heading towards a new L-shape heading (in place), wrap-safe.
    alpha: weight of the new scan. Kept high: heavier smoothing lags in corners (yaw rate ~1 rad/s).
    """
    if not np.isfinite(heading):
        return
    if not np.isfinite(tr.heading):
        tr.heading = heading
        return
    step = np.arctan2(np.sin(heading - tr.heading), np.cos(heading - tr.heading))
    tr.heading = float(np.arctan2(np.sin(tr.heading + alpha * step), np.cos(tr.heading + alpha * step)))


def inside_footprint(a_s, a_d, b_s, b_d, track_length, length=0.625, width=0.326) -> bool:
    """Whether position a lies within one car footprint (max legal length x width) of position b."""
    return abs(wrap_ds(a_s - b_s, track_length)) < length and abs(a_d - b_d) < width

def step_tracks(
        tracks,
        observations,
        t,
        track_length,
        next_id,
        std_s=0.03,
        std_d=0.02,
        gate=9.21,
        coast_max=0.7,
        max_tracks=8,
        merge_length=0.625,
        merge_width=0.326,
        heading_alpha=0.7,
        t_static=0.75,
        static_spread=0.05,
        ego=None,
        t_fast=0.3,
        path_half_width=0.25,
        fast_range=6.0):
    """
    Advance all tracks to time t and fold in this scan's observations.
    Each track turns an observation into its own measurement (anchor) with the shape it learned,
    so the same observation can mean slightly different rear positions for different tracks.
    Returns the updated list of tracks and the next free track id.
    """

    def measure(obs, shape):
        return anchor(obs, shape, track_length, std_s=std_s, std_d=std_d)

    def needed_still_time(tr, obs):
        """Fast path: an object close ahead in our path is confirmed static after t_fast, everything else t_static."""
        if ego is not None and in_our_path(tr, obs, ego[0], ego[1], track_length, path_half_width, fast_range):
            return t_fast
        return t_static

    # Predict every track to this scan's time
    for tr in tracks:
        dt = t - tr.t
        if dt > 0:
            tr.x, tr.P = kf_predict(tr.x, tr.P, dt, track_length)
            tr.t = t

    # All track-observation pairs inside the gate, best first
    pairs = []
    for ti, tr in enumerate(tracks):
        for oi, obs in enumerate(observations):
            z, R = measure(obs, tr.shape)
            nis = nis_of(tr.x, tr.P, z, R, track_length)
            if nis < gate:
                pairs.append((nis, ti, oi))
    pairs.sort()

    used_tracks, used_obs = set(), set()
    for nis, ti, oi in pairs:
        if ti in used_tracks or oi in used_obs:
            continue
        used_tracks.add(ti)
        used_obs.add(oi)

        # Update with the assigned observation, then learn the shape from it
        tr = tracks[ti]
        z, R = measure(observations[oi], tr.shape)
        tr.x, tr.P = kf_update(tr.x, tr.P, z, R, track_length)
        update_shape(tr.shape, observations[oi])
        update_heading(tr, observations[oi].heading, heading_alpha)
        update_static_evidence(tr, observations[oi], t, track_length, t_static, static_spread,
                               needed_still_time(tr, observations[oi]))
        tr.t_last_hit = t

    # 3./4. Record hit or miss, keep the last 4 scans, confirm at 3 of 4
    for ti, tr in enumerate(tracks):
        tr.hits = (tr.hits + [ti in used_tracks])[-4:]
        if not tr.confirmed and sum(tr.hits) >= 3:
            tr.confirmed = True

    # 5. Delete: unconfirmed that can't reach 3 of 4 anymore, confirmed that were missing too long
    kept = []
    for tr in tracks:
        if not tr.confirmed and tr.hits.count(False) >= 2:
            continue
        if tr.confirmed and t - tr.t_last_hit > coast_max:
            continue
        kept.append(tr)
    tracks = kept

    # Merge tracks inside an older track's car footprint (one car can't be two tracks): keep the older one
    tracks.sort(key=lambda tr: tr.id)
    merged = []
    for tr in tracks:
        if not any(inside_footprint(tr.x[0], tr.x[1], o.x[0], o.x[1], track_length, merge_length, merge_width)
                   for o in merged):
            merged.append(tr)
    tracks = merged

    # 6. Start a new track for each observation nobody claimed (unless a track is already right there)
    for oi, obs in enumerate(observations):
        z, R = measure(obs, Shape())
        near = any(inside_footprint(z[0], z[1], tr.x[0], tr.x[1], track_length, merge_length, merge_width)
                   for tr in tracks)
        if oi not in used_obs and not near and len(tracks) < max_tracks:
            tr = new_track(next_id, z, t, R)
            update_shape(tr.shape, obs)
            update_heading(tr, obs.heading, heading_alpha)
            update_static_evidence(tr, obs, t, track_length, t_static, static_spread, needed_still_time(tr, obs))
            tracks.append(tr)
            next_id += 1

    # Static or dynamic, only learned from scans where the track was actually detected
    # (a coasting track keeps its label; otherwise its extrapolated motion could make a static object "dynamic")
    for tr in tracks:
        if tr.t_last_hit == t:
            update_motion_class(tr, track_length)

    return tracks, next_id

def update_static_evidence(tr: Track, obs: Observation, t, track_length, t_static=0.75, spread_max=0.05,
                           t_needed=None) -> None:
    """
    A track is a static object if its detected position has stayed within spread_max for at least t_needed seconds
    (default t_static; shorter for an object in our path, see in_our_path). The window always covers t_static.
    The observed bounding boxes are accumulated, gradually building the obstacle as it is seen.
    """
    # The one position history of the track (also read by update_motion_class); keeps t_static + margin
    tr.history.append((t, tr.x[0], tr.x[1]))
    tr.history = [x for x in tr.history if t - x[0] <= t_static + 0.3]
    h = np.array(tr.history)

    # Spread of the position over the window (s relative to the newest, wrap-safe)
    ds = wrap_ds(h[:, 1] - h[-1, 1], track_length)
    moved = np.std(ds) >= spread_max or np.std(h[:, 2]) >= spread_max
    if moved:
        tr.static_confirmed, tr.extent = False, None
        return
    
    # Once confirmed it stays static until it moves
    tr.static_confirmed = bool(tr.static_confirmed or h[-1, 0] - h[0, 0] >= (t_static if t_needed is None else t_needed))

    # Accumulate this scan's box in ABSOLUTE coordinates: a static object's outline must not move with the track
    # position, which drifts when the object is seen from other sides (e.g. from the front once we have passed it)
    if np.isfinite(obs.length) and np.isfinite(obs.width):
        lo = (obs.s_c - obs.length / 2) % track_length
        box_d = (obs.d_c - obs.width / 2, obs.d_c + obs.width / 2)
        if tr.extent is None:
            tr.extent = (lo, lo + obs.length, *box_d)
        else:
            tr.extent = union_box(tr.extent, lo, obs.length, *box_d, track_length)


def in_our_path(tr: Track, obs: Observation, ego_s, ego_d, track_length, half_width=0.25, max_range=6.0) -> bool:
    """Is the object close ahead (< max_range) and does its lateral extent overlap +-half_width around the
    raceline or around our current lateral position?"""
    gap = wrap_ds(tr.x[0] - ego_s, track_length)
    if not 0.0 < gap < max_range:
        return False
    d_lo, d_hi = tr.x[1] - MIN_FACE / 2, tr.x[1] + MIN_FACE / 2
    if np.isfinite(obs.width):
        d_lo, d_hi = min(d_lo, obs.d_c - obs.width / 2), max(d_hi, obs.d_c + obs.width / 2)
    if tr.extent is not None:
        d_lo, d_hi = min(d_lo, tr.extent[2]), max(d_hi, tr.extent[3])
    return any(d_lo < line + half_width and d_hi > line - half_width for line in (0.0, ego_d))


def union_box(extent, lo, length, d_lo, d_hi, track_length) -> tuple[float, float, float, float]:
    """Grow an absolute box (s_lo, s_hi, d_lo, d_hi) by another one starting at s = lo (wrap-safe)."""
    e_lo, e_hi, e_dlo, e_dhi = extent
    lo = e_lo + wrap_ds(lo - e_lo, track_length)          # the new start, unwrapped next to the stored one
    return min(e_lo, lo), max(e_hi, lo + length), min(e_dlo, d_lo), max(e_dhi, d_hi)


def static_extent(tr: Track, track_length) -> tuple[float, float, float, float]:
    """The accumulated box in absolute Frenet coordinates: (s_start, s_end, d_right, d_left)."""
    lo, hi, d_lo, d_hi = tr.extent
    return lo % track_length, hi % track_length, d_lo, d_hi
