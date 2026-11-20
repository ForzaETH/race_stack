"""
Opponent tracking in Frenet coordinates. Pure Python, no ROS.

State x = [s, d, vs, vd]: position along / across the raceline [m] and their speeds [m/s]
Constant velocity Kalman filter.
"""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class Track:
    """One tracked object."""
    id: int
    x: np.ndarray
    P: np.ndarray
    t: float
    t_last_hit: float       # last time a detection was assigned
    hits: list = field(default_factory=list)
    confirmed: bool = False
    dynamic: bool = False
    history: list = field(default_factory=list)   # recent (t, s, d) of the estimate, for static/dynamic


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
    Returns the new state, its uncertainty, and the NIS (how surprising the measurement was).
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
    nis = float(y @ np.linalg.inv(S) @ y)
    return x, P, nis


def nis_of(x, P, z, R, track_length) -> float:
    """How well detection z fits a track's predicted position."""
    y = np.asarray(z, float) - x[:2]
    y[0] = wrap_ds(y[0], track_length)
    S = P[:2, :2] + R
    return float(y @ np.linalg.inv(S) @ y)


def update_motion_class(tr: Track, track_length, v_up=0.5, v_down=0.2, window=0.5, min_span=0.3) -> None:
    """
    Static / dynamic from how far the track moved over the last `window` seconds (averages out the noisy
    speed estimate). Hysteresis: dynamic above v_up, static again below v_down [m/s].
    """
    tr.history.append((tr.t, tr.x[0], tr.x[1]))
    tr.history = [h for h in tr.history if tr.t - h[0] <= window]

    span = tr.history[-1][0] - tr.history[0][0]
    if span < min_span:
        return                                     # not enough history yet: keep the current label

    ds = wrap_ds(tr.history[-1][1] - tr.history[0][1], track_length)
    dd = tr.history[-1][2] - tr.history[0][2]
    speed = np.hypot(ds, dd) / span
    if not tr.dynamic and speed > v_up:
        tr.dynamic = True
    elif tr.dynamic and speed < v_down:
        tr.dynamic = False


def step_tracks(
        tracks,
        detections,
        t,
        R,
        track_length,
        next_id,
        gate=9.21,
        coast_max=0.7,
        max_tracks=8,
        spawn_dist=0.6,
        merge_dist=0.3):
    """
    Advance all tracks to time t and fold in this scan's detections.
    Returns the updated list of track and the next free track id.
    """

    # Predict every track to this scan's time
    for tr in tracks:
        dt = t - tr.t
        if dt > 0:
            tr.x, tr.P = kf_predict(tr.x, tr.P, dt, track_length)
            tr.t = t

    # All track detection pairs inside the gate, best first
    pairs = []
    for ti, tr in enumerate(tracks):
        for di, z in enumerate(detections):
            nis = nis_of(tr.x, tr.P, z, R, track_length)
            if nis < gate:
                pairs.append((nis, ti, di))
    pairs.sort()

    used_tracks, used_dets = set(), set()
    for nis, ti, di in pairs:
        if ti in used_tracks or di in used_dets:
            continue
        used_tracks.add(ti)
        used_dets.add(di)
        # 3. Update with the assigned detection
        tr = tracks[ti]
        tr.x, tr.P, _ = kf_update(tr.x, tr.P, np.asarray(detections[di], float), R, track_length)
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

    # Merge tracks that converged onto the same object: keep the older one (lower id)
    tracks.sort(key=lambda tr: tr.id)
    merged = []
    for tr in tracks:
        too_close = any(np.hypot(wrap_ds(tr.x[0] - o.x[0], track_length), tr.x[1] - o.x[1]) < merge_dist
                        for o in merged)
        if not too_close:
            merged.append(tr)
    tracks = merged

    # 6. Start a new track for each detection nobody claimed (unless a track is already right there)
    for di, z in enumerate(detections):
        near = any(np.hypot(wrap_ds(z[0] - tr.x[0], track_length), z[1] - tr.x[1]) < spawn_dist
                   for tr in tracks)
        if di not in used_dets and not near and len(tracks) < max_tracks:
            tracks.append(new_track(next_id, z, t, R))
            next_id += 1

    # Static or dynamic, for every track
    for tr in tracks:
        update_motion_class(tr, track_length)

    return tracks, next_id
