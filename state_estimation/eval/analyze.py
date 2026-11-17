#!/usr/bin/env python3
"""Offline analysis of a state estimation replay bag.

Reads the bag recorded by run_eval.py (or any bag with the topics below) with
`rosbags`, no ROS needed. Produces plots/, data/ (TUM + CSV + metrics.json) and
summary.md in the result folder.

  global  : cartographer map->base_link (/tracked_pose, and map->odom from /tf)
  local   : EKF /state_estimation/odom, RPE against cartographer (or ground truth)
  inputs  : /odom (VESC), /imu, /basalt/odom
  gt      : optional --gt-topic (PoseStamped / Odometry / TransformStamped /
            PoseWithCovarianceStamped). APE/RPE are then computed against it.
"""

import argparse
import json
import math
from pathlib import Path

import matplotlib
try:
    from eth_colors_pbl import ETHColors as _ETH
    _eth = {k: _ETH[k].value for k in ('ETH_BLUE', 'ETH_PETROL', 'ETH_GREEN', 'ETH_BRONZE', 'ETH_RED', 'ETH_PURPLE', 'ETH_GRAY', 'ETH_GRAY_DARK')}
except ImportError:   # pip install eth-colors-pbl; same values so the plots look the same without it
    _eth = {'ETH_BLUE': '#215caf', 'ETH_PETROL': '#007894', 'ETH_GREEN': '#627313', 'ETH_BRONZE': '#8e6713',
            'ETH_RED': '#b7352d', 'ETH_PURPLE': '#a7117a', 'ETH_GRAY': '#6f6f6f', 'ETH_GRAY_DARK': '#575757'}
matplotlib.use('Agg')
# one colour per source in every plot (ETH palette)
COL = {'gt': _eth['ETH_GREEN'], 'carto': _eth['ETH_GRAY_DARK'], 'ekf': _eth['ETH_RED'], 'basalt': _eth['ETH_PETROL'],
       'vesc': _eth['ETH_BLUE'], 'imu': _eth['ETH_BRONZE'], 'alt': _eth['ETH_PURPLE']}
matplotlib.rcParams['axes.prop_cycle'] = matplotlib.cycler(
    color=[_eth[k] for k in ('ETH_BLUE', 'ETH_RED', 'ETH_PETROL', 'ETH_GREEN', 'ETH_BRONZE', 'ETH_PURPLE', 'ETH_GRAY')])
import matplotlib.pyplot as plt
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.signal import savgol_filter

TOPICS = {
    'carto': '/tracked_pose',
    'ekf': '/state_estimation/odom',
    'basalt': '/basalt/odom',
    'vesc': '/odom',
    'imu': '/imu',
    'tf': '/tf',
}
RPE_DELTA_S = 1.0
GT_MAX_DT = 0.02
MOVE_SPEED = 0.5      # m/s, first time the car is faster than this = start of motion
WARMUP_S = 3.0        # s after start of motion before the evaluation window begins (cartographer init)
VEL_WINDOW_S = 0.25   # s, Savitzky-Golay window of the reference velocities (0.15 left 0.045 m/s jitter on Vicon vy, 0.25 halves it)
YAW_WINDOW_S = 0.3    # s, smoothing of the reference heading before rotating velocities into the body frame
INIT_S = 10.0         # s after the first map->odom TF in which a jump counts as initialisation, not as a loss
EXPECTED = ('carto', 'ekf', 'basalt', 'vesc', 'imu', 'tf')
RED = '\033[1;91m'
END = '\033[0m'


def alert(msg):
    bar = '!' * 78
    print(f'{RED}{bar}\n{msg}\n{bar}{END}', flush=True)


# ----------------------------------------------------------------------------
# bag reading
# ----------------------------------------------------------------------------
def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose_of(msg):
    """Position/orientation of PoseStamped, Odometry, PoseWithCovarianceStamped, TransformStamped."""
    if hasattr(msg, 'transform'):
        return msg.transform.translation, msg.transform.rotation
    p = msg.pose
    if hasattr(p, 'pose'):
        p = p.pose
    return p.position, p.orientation


def read_bag(path, topics):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    out = {t: [] for t in topics}
    with Reader(path) as r:
        conns = [c for c in r.connections if c.topic in topics]
        for c, t_recv, raw in r.messages(connections=conns):
            out[c.topic].append((t_recv * 1e-9, ts.deserialize_cdr(raw, c.msgtype)))
    return out


def poses(msgs):
    """-> dict of arrays t, t_recv, x, y, yaw (+ twist for Odometry)."""
    if not msgs:
        return None
    d = {'t': [], 't_recv': [], 'x': [], 'y': [], 'yaw': []}
    has_twist = hasattr(msgs[0][1], 'twist')
    if has_twist:
        d.update(vx=[], vy=[], vyaw=[])
    for t_recv, m in msgs:
        p, q = pose_of(m)
        d['t'].append(stamp(m.header)); d['t_recv'].append(t_recv)
        d['x'].append(p.x); d['y'].append(p.y); d['yaw'].append(yaw_of(q))
        if has_twist:
            d['vx'].append(m.twist.twist.linear.x)
            d['vy'].append(m.twist.twist.linear.y)
            d['vyaw'].append(m.twist.twist.angular.z)
    d = {k: np.array(v) for k, v in d.items()}
    order = np.argsort(d['t'], kind='stable')
    return {k: v[order] for k, v in d.items()}


def imu(msgs):
    if not msgs:
        return None
    d = {'t': [], 't_recv': [], 'gx': [], 'gy': [], 'gz': [], 'ax': [], 'ay': [], 'az': [], 'yaw': []}
    for t_recv, m in msgs:
        d['t'].append(stamp(m.header)); d['t_recv'].append(t_recv)
        w, a = m.angular_velocity, m.linear_acceleration
        d['gx'].append(w.x); d['gy'].append(w.y); d['gz'].append(w.z)
        d['ax'].append(a.x); d['ay'].append(a.y); d['az'].append(a.z)
        d['yaw'].append(yaw_of(m.orientation))
    return {k: np.array(v) for k, v in d.items()}


def tf_pairs(msgs, parent, child):
    d = {'t': [], 't_recv': [], 'x': [], 'y': [], 'yaw': []}
    for t_recv, m in msgs:
        for tr in m.transforms:
            if tr.header.frame_id == parent and tr.child_frame_id == child:
                d['t'].append(stamp(tr.header)); d['t_recv'].append(t_recv)
                d['x'].append(tr.transform.translation.x)
                d['y'].append(tr.transform.translation.y)
                d['yaw'].append(yaw_of(tr.transform.rotation))
    d = {k: np.array(v) for k, v in d.items()}
    if len(d['t']) == 0:
        return None
    order = np.argsort(d['t'], kind='stable')
    return {k: v[order] for k, v in d.items()}


def compose(a, b):
    """SE2 chain a(parent->mid) * b(mid->child), a sampled at b's stamps (latest a <= t)."""
    idx = np.clip(np.searchsorted(a['t'], b['t'], side='right') - 1, 0, len(a['t']) - 1)
    c, s = np.cos(a['yaw'][idx]), np.sin(a['yaw'][idx])
    return {
        't': b['t'], 't_recv': b['t_recv'],
        'x': a['x'][idx] + c * b['x'] - s * b['y'],
        'y': a['y'][idx] + s * b['x'] + c * b['y'],
        'yaw': wrap(a['yaw'][idx] + b['yaw']),
    }


# ----------------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------------
def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def body_velocities(tr, hz=100.0, window_s=VEL_WINDOW_S, yaw_window_s=YAW_WINDOW_S):
    """vx, vy, vyaw in body frame by differentiating a pose trajectory (resampled, Savitzky-Golay).
    The heading used to rotate into the body frame is smoothed too: its jitter (~0.4 deg for Vicon) leaks
    v * sin(jitter) of the speed into vy, 0.03 m/s per sample at 4 m/s."""
    t = np.arange(tr['t'][0], tr['t'][-1], 1.0 / hz)
    if len(t) < 10:
        return None
    x = np.interp(t, tr['t'], tr['x'])
    y = np.interp(t, tr['t'], tr['y'])
    yaw_raw = np.interp(t, tr['t'], np.unwrap(tr['yaw']))
    win = max(5, int(window_s * hz) | 1)
    wy = max(5, int(yaw_window_s * hz) | 1)
    yaw = savgol_filter(yaw_raw, wy, 3)
    dx = savgol_filter(x, win, 3, deriv=1, delta=1.0 / hz)
    dy = savgol_filter(y, win, 3, deriv=1, delta=1.0 / hz)
    vyaw = savgol_filter(yaw_raw, wy, 3, deriv=1, delta=1.0 / hz)
    c, s = np.cos(yaw), np.sin(yaw)
    out = {'t': t, 'vx': c * dx + s * dy, 'vy': -s * dx + c * dy, 'vyaw': vyaw, 'yaw': wrap(yaw), 'src': 'diff.'}
    nan_gaps(out, tr['t'], win)
    return out


def gap_mask(t, ts, factor=3.0, min_gap_s=0.1):
    """True where the grid t lies inside a gap of the source stamps ts (dt > factor x median and > min_gap_s:
    tracking lost, not a skipped scan)."""
    dt = np.diff(ts)
    bad = np.zeros(len(t), bool)
    for i in np.flatnonzero(dt > max(factor * np.median(dt), min_gap_s)):
        bad |= (t >= ts[i]) & (t <= ts[i + 1])
    return bad


def nan_gaps(v, ts, win):
    """Set the velocities of v to NaN inside gaps of ts, widened by the smoothing window (the interpolated
    values there are fiction; NaNs are dropped by stats() and leave holes in the plots)."""
    bad = gap_mask(v['t'], ts)
    if bad.any():
        bad = np.convolve(bad, np.ones(win), 'same') > 0
        for k in ('vx', 'vy', 'vyaw'):
            v[k][bad] = np.nan
        v['gaps'] = int(np.diff(np.r_[0, bad.astype(int), 0]).clip(0).sum())


def twists(msgs):
    """TwistStamped -> dict of arrays t, vx, vy, vz, wz (frame as published)."""
    if not msgs:
        return None
    d = {k: [] for k in ('t', 't_recv', 'vx', 'vy', 'vz', 'wz')}
    for t_recv, m in msgs:
        tw = m.twist.twist if hasattr(m.twist, 'twist') else m.twist
        d['t'].append(stamp(m.header)); d['t_recv'].append(t_recv)
        d['vx'].append(tw.linear.x); d['vy'].append(tw.linear.y); d['vz'].append(tw.linear.z); d['wz'].append(tw.angular.z)
    return {k: np.asarray(v) for k, v in d.items()}


def twist_body(tw, tr, hz=100.0, window_s=VEL_WINDOW_S, yaw_window_s=YAW_WINDOW_S):
    """Published (world-frame) GT twist rotated into the body frame with the GT yaw, resampled and smoothed
    like body_velocities() so the two references are comparable.
    The Vicon bridge stamps a backward difference with the end of its interval, so the twist is valid half a
    sample earlier; at 2.4 rad/s those ~8 ms are 1.1 deg of yaw, i.e. 0.07 m/s leaking from vx into vy.
    The yaw rate is taken from the differentiated pose: the bridge's angular.z is clipped at +-pi and reads
    ~20 % low above 2.8 rad/s (glc_big_bangbang, glc_oval), the pose derivative tracks the gyro."""
    pv = body_velocities(tr, hz, window_s, yaw_window_s)
    dt = np.diff(tw['t'])
    bad = np.r_[False, dt > 1.5 * np.median(dt)]   # the sample after a tracking gap differences across the gap
    tw = dict(tw, t=tw['t'] - 0.5 * np.median(dt), vx=np.where(bad, np.nan, tw['vx']), vy=np.where(bad, np.nan, tw['vy']))
    t = np.arange(max(tw['t'][0], tr['t'][0]), min(tw['t'][-1], tr['t'][-1]), 1.0 / hz)
    if len(t) < 10 or pv is None:
        return None
    win = max(5, int(window_s * hz) | 1)
    vxw = savgol_filter(np.interp(t, tw['t'], tw['vx']), win, 3)
    vyw = savgol_filter(np.interp(t, tw['t'], tw['vy']), win, 3)
    yaw = savgol_filter(np.interp(t, tr['t'], np.unwrap(tr['yaw'])), max(5, int(yaw_window_s * hz) | 1), 3)
    c, s = np.cos(yaw), np.sin(yaw)
    out = {'t': t, 'vx': c * vxw + s * vyw, 'vy': -s * vxw + c * vyw, 'vyaw': np.interp(t, pv['t'], pv['vyaw']),
           'yaw': wrap(yaw), 'src': 'twist'}
    nan_gaps(out, tw['t'], win)
    return out


def time_offset(ta, va, tb, vb, hz=200.0, max_lag=0.6):
    """Seconds by which signal a leads signal b (cross-correlation peak). >0: a's stamps are early."""
    t = np.arange(max(ta[0], tb[0]), min(ta[-1], tb[-1]), 1.0 / hz)
    if len(t) < 4 * hz:
        return None
    a = np.interp(t, ta, va); b = np.interp(t, tb, vb)
    ok = np.isfinite(a) & np.isfinite(b)   # masked gaps: zero after mean removal, they drop out of the dot products
    if ok.sum() < 4 * hz or np.std(a[ok]) < 1e-6 or np.std(b[ok]) < 1e-6:
        return None
    a = np.where(ok, a - a[ok].mean(), 0.0); b = np.where(ok, b - b[ok].mean(), 0.0)
    n = int(max_lag * hz)
    scores = [np.dot(a[k:], b[:len(b) - k]) if k >= 0 else np.dot(a[:k], b[-k:]) for k in range(-n, n + 1)]
    return -float((np.argmax(scores) - n) / hz)


def crop(d, t_start):
    if d is None:
        return None
    keep = d['t'] >= t_start
    return {k: v[keep] for k, v in d.items()}


def stats(e):
    e = np.asarray(e, dtype=float)
    e = e[np.isfinite(e)]
    if len(e) == 0:
        return {}
    return {'rmse': float(np.sqrt(np.mean(e ** 2))), 'mean': float(np.mean(e)),
            'median': float(np.median(e)), 'std': float(np.std(e)),
            'max': float(np.max(np.abs(e))), 'n': int(len(e))}


def rate_stats(d):
    dt = np.diff(d['t'])
    lat = d['t_recv'] - d['t']
    return {'rate_hz': float(1.0 / np.median(dt)) if len(dt) else 0.0,
            'dt_max_s': float(dt.max()) if len(dt) else 0.0,
            'non_monotonic': int((dt < 0).sum()),
            'latency_median_s': float(np.median(lat)), 'latency_max_s': float(lat.max())}


def interp_to(t, d, key, angular=False):
    v = d[key]
    if angular:
        return wrap(np.interp(t, d['t'], np.unwrap(v)))
    return np.interp(t, d['t'], v)


def to_evo(tr):
    from evo.core.trajectory import PoseTrajectory3D
    n = len(tr['t'])
    q = np.zeros((n, 4))
    q[:, 0] = np.cos(tr['yaw'] / 2)
    q[:, 3] = np.sin(tr['yaw'] / 2)
    xyz = np.column_stack([tr['x'], tr['y'], np.zeros(n)])
    return PoseTrajectory3D(positions_xyz=xyz, orientations_quat_wxyz=q, timestamps=tr['t'])


def write_tum(path, tr):
    q = np.column_stack([np.zeros(len(tr['t'])), np.zeros(len(tr['t'])),
                         np.sin(tr['yaw'] / 2), np.cos(tr['yaw'] / 2)])
    np.savetxt(path, np.column_stack([tr['t'], tr['x'], tr['y'], np.zeros(len(tr['t'])), q]),
               fmt='%.6f', header='t x y z qx qy qz qw', comments='# ')


def write_csv(path, d):
    keys = list(d.keys())
    np.savetxt(path, np.column_stack([d[k] for k in keys]), fmt='%.6f',
               delimiter=',', header=','.join(keys), comments='')


# ----------------------------------------------------------------------------
# evo metrics
# ----------------------------------------------------------------------------
def resample(tr, t):
    return {'t': t, 'x': np.interp(t, tr['t'], tr['x']), 'y': np.interp(t, tr['t'], tr['y']),
            'yaw': interp_to(t, tr, 'yaw', angular=True)}


def evo_rpe(ref, est, hz=50.0):
    """Relative pose error est vs ref over RPE_DELTA_S. Frame-independent -> local accuracy.
    evo has no time delta unit, so both are resampled to hz and delta is given in frames."""
    from evo.core import metrics
    try:
        from evo.core.units import Unit
    except ImportError:                 # evo < 1.20
        from evo.core.metrics import Unit
    t = np.arange(max(ref['t'][0], est['t'][0]), min(ref['t'][-1], est['t'][-1]), 1.0 / hz)
    r, e = to_evo(resample(ref, t)), to_evo(resample(est, t))
    out = {}
    for name, rel in (('trans_m', metrics.PoseRelation.translation_part),
                      ('rot_deg', metrics.PoseRelation.rotation_angle_deg)):
        m = metrics.RPE(rel, int(round(RPE_DELTA_S * hz)), Unit.frames, all_pairs=False)
        m.process_data((r, e))
        out[name] = {'t': t[m.delta_ids], 'err': m.error, 'stats': stats(m.error)}
    return out


def evo_ape(ref, est, max_dt, align=True):
    from evo.core import metrics, sync
    r, e = sync.associate_trajectories(to_evo(ref), to_evo(est), max_diff=max_dt)
    if align:
        e.align(r, correct_scale=False)
    out = {}
    for name, rel in (('trans_m', metrics.PoseRelation.translation_part),
                      ('rot_deg', metrics.PoseRelation.rotation_angle_deg)):
        m = metrics.APE(rel)
        m.process_data((r, e))
        out[name] = {'t': e.timestamps, 'err': m.error, 'stats': stats(m.error)}
    out['aligned'] = {'t': e.timestamps, 'x': e.positions_xyz[:, 0], 'y': e.positions_xyz[:, 1]}
    return out


# ----------------------------------------------------------------------------
# plots
# ----------------------------------------------------------------------------
def finish(fig, P, title, fname):
    fig.suptitle(title, fontsize=14, fontweight='bold')
    fig.tight_layout(); fig.savefig((P['plots'] / fname).with_suffix('.pdf')); plt.close(fig)   # vector, viewable in VS Code with a PDF extension


def plot_trajectories(P, carto, ekf, basalt, gt_ape, gt=None):
    fig, ax = plt.subplots(1, 3 if gt_ape else 2, figsize=(18, 6))
    if gt_ape:
        # ground truth frame: GT in green, cartographer rigidly aligned onto it (Umeyama, no scale)
        ax[0].plot(gt['x'], gt['y'], lw=1.2, color=COL['gt'], label='ground truth')
        ax[0].plot(gt_ape['aligned']['x'], gt_ape['aligned']['y'], lw=0.8, color=COL['carto'], label='cartographer, aligned to GT')
        ax[0].set_title('Ground truth frame')
    else:
        ax[0].plot(carto['x'], carto['y'], lw=0.8, color=COL['carto'], label='cartographer map->base_link')
        ax[0].set_title('Cartographer, map frame')
    ax[0].set_xlabel('x [m]'); ax[0].set_ylabel('y [m]'); ax[0].axis('equal'); ax[0].legend()
    if ekf:
        ax[1].plot(ekf['x'], ekf['y'], lw=0.8, color=COL['ekf'], label='EKF /state_estimation/odom')
    if basalt:
        ax[1].plot(basalt['x'], basalt['y'], lw=0.8, color=COL['basalt'], label='/basalt/odom')
    ax[1].set_title('Odometry sources, odom frame'); ax[1].set_xlabel('x [m]'); ax[1].set_ylabel('y [m]'); ax[1].axis('equal'); ax[1].legend()
    if gt_ape:
        ax[2].plot(gt_ape['trans_m']['t'] - P['t0'], gt_ape['trans_m']['err'], color=COL['carto'])
        ax[2].set_title(f'Cartographer APE vs ground truth, rmse {gt_ape["trans_m"]["stats"]["rmse"]:.3f} m')
        ax[2].set_xlabel('t [s]'); ax[2].set_ylabel('error [m]')
    finish(fig, P, 'Trajectories', 'trajectories.png')


def lap_bounds(ref, r=0.5):
    """Start times of the laps of a closed track: every return to within r of the first position of the window after
    having been more than 3 r away. The window starts wherever the car was, so "lap" = that point to that point."""
    d = np.hypot(ref['x'] - ref['x'][0], ref['y'] - ref['y'][0])
    bounds, away = [ref['t'][0]], False
    for i in range(len(d)):
        if d[i] > 3 * r:
            away = True
        elif away and d[i] < r:
            bounds.append(ref['t'][i]); away = False
    return bounds + [ref['t'][-1]]


def window(d, t0, t1):
    keep = (d['t'] >= t0) & (d['t'] <= t1)
    return {k: v[keep] for k, v in d.items() if isinstance(v, np.ndarray) and len(v) == len(keep)}


def plot_laps(P, ref, ref_name, sources):
    """plots/laps/trajectory_<i>.pdf: one plot per lap, each odometry source aligned (Umeyama, no scale) onto the
    reference over that lap only, so the per-lap drift of EKF / Basalt is visible. Returns per-lap stats."""
    bounds = lap_bounds(ref)
    laps = []
    (P['plots'] / 'laps').mkdir(exist_ok=True)
    for i in range(len(bounds) - 1):
        t0, t1 = bounds[i], bounds[i + 1]
        if t1 - t0 < 1.0:
            continue
        rl = window(ref, t0, t1)
        fig, ax = plt.subplots(figsize=(9, 8))
        ax.plot(rl['x'], rl['y'], lw=1.5, color=COL[ref_name], label=ref_name.replace('gt', 'ground truth').replace('carto', 'cartographer'))
        lap = {'lap': i, 't_start_s': float(t0 - P['t0']), 't_end_s': float(t1 - P['t0'])}
        for name, d in sources:
            if d is None:
                continue
            dl = window(d, t0, t1)
            if len(dl['t']) < 10:
                continue
            ape = evo_ape(rl, dl, GT_MAX_DT, align=True)
            lap[f'{name}_ape_m'] = ape['trans_m']['stats']['rmse']
            ax.plot(ape['aligned']['x'], ape['aligned']['y'], lw=0.9, color=COL[name],
                    label=f'{name.replace("carto", "cartographer").replace("ekf", "EKF").replace("basalt", "Basalt")}, aligned, APE {ape["trans_m"]["stats"]["rmse"]:.3f} m')
        ax.set_title(f'Lap {i}: t = {t0 - P["t0"]:.1f} .. {t1 - P["t0"]:.1f} s ({t1 - t0:.1f} s)')
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.axis('equal'); ax.legend(loc='upper right')
        finish(fig, P, 'Trajectory per lap', f'laps/trajectory_{i}.png')
        laps.append(lap)
    return laps


def plot_global(P, map_odom):
    fig, ax = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    t = map_odom['t'] - P['t0']
    ax[0].plot(t, map_odom['x'], lw=0.8); ax[0].set_title('map → odom x'); ax[0].set_ylabel('[m]')
    ax[1].plot(t, map_odom['y'], lw=0.8); ax[1].set_title('map → odom y'); ax[1].set_ylabel('[m]')
    ax[2].plot(t, np.degrees(np.unwrap(map_odom['yaw'])), lw=0.8)
    ax[2].set_title('map → odom yaw'); ax[2].set_ylabel('[deg]'); ax[2].set_xlabel('t [s]')
    finish(fig, P, 'Global consistency', 'global_consistency.png')


def plot_rpe(P, rpe, title, fname, color=COL['ekf']):
    fig, ax = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    for a, k, lab in ((ax[0], 'trans_m', 'translation [m]'), (ax[1], 'rot_deg', 'rotation [deg]')):
        a.plot(rpe[k]['t'] - P['t0'], rpe[k]['err'], lw=0.8, color=color)
        s = rpe[k]['stats']
        a.set_ylabel(lab); a.set_title(f'{lab.split(" ")[0].capitalize()}: rmse {s.get("rmse", 0):.3f}, max {s.get("max", 0):.3f}')
    ax[1].set_xlabel('t [s]')
    finish(fig, P, title, fname)


def plot_velocities(P, ekf, vesc, basalt, imu_d, carto_v, gt_v):
    fig, ax = plt.subplots(3, 1, figsize=(14, 11), sharex=True)
    t0 = P['t0']

    def line(a, d, key, label, **kw):
        if d is not None and key in d:
            a.plot(d['t'] - t0, d[key], lw=0.7, label=label, **kw)

    line(ax[0], carto_v, 'vx', 'cartographer (diff.)', color=COL['carto'], alpha=0.5)
    gt_lab = 'ground truth (' + (gt_v['src'] if gt_v else '') + ')'
    line(ax[0], gt_v, 'vx', gt_lab, color=COL['gt'])
    line(ax[0], vesc, 'vx', '/odom (VESC)', color=COL['vesc'])
    line(ax[0], basalt, 'vx', '/basalt/odom', color=COL['basalt'])
    line(ax[0], ekf, 'vx', 'EKF', color=COL['ekf'])
    ax[0].set_title('Longitudinal velocity'); ax[0].set_ylabel('vx [m/s]'); ax[0].legend(loc='upper right')
    line(ax[1], carto_v, 'vy', 'cartographer (diff.)', color=COL['carto'], alpha=0.5)
    line(ax[1], gt_v, 'vy', gt_lab, color=COL['gt'])
    line(ax[1], basalt, 'vy', '/basalt/odom', color=COL['basalt'])
    line(ax[1], ekf, 'vy', 'EKF', color=COL['ekf'])
    ax[1].set_title('Lateral velocity'); ax[1].set_ylabel('vy [m/s]'); ax[1].legend(loc='upper right')
    line(ax[2], carto_v, 'vyaw', 'cartographer (diff.)', color=COL['carto'], alpha=0.5)
    line(ax[2], gt_v, 'vyaw', gt_lab, color=COL['gt'])
    line(ax[2], imu_d, 'gz', '/imu gyro z', color=COL['imu'])
    line(ax[2], basalt, 'vyaw', '/basalt/odom', color=COL['basalt'])
    line(ax[2], ekf, 'vyaw', 'EKF', color=COL['ekf'])
    ax[2].set_title('Yaw rate'); ax[2].set_ylabel('[rad/s]'); ax[2].legend(loc='upper right'); ax[2].set_xlabel('t [s]')
    finish(fig, P, 'Velocities: inputs, EKF and reference', 'velocities.png')


def plot_yaw(P, carto, ekf, basalt, imu_d):
    fig, ax = plt.subplots(1, 1, figsize=(14, 5))
    ax = [ax]
    t0 = P['t0']
    cy = np.unwrap(carto['yaw'])
    ax[0].plot(carto['t'] - t0, np.degrees(carto['yaw']), lw=0.8, label='cartographer (map)', color=COL['carto'])
    for d, lab, col in ((ekf, 'EKF (odom)', COL['ekf']), (basalt, '/basalt/odom', COL['basalt']), (imu_d, '/imu orientation', COL['imu'])):
        if d is not None:
            # odom-frame yaws start at the cartographer yaw so they are comparable; drift shows as divergence
            y = np.unwrap(d['yaw'])
            y = y - y[0] + np.interp(d['t'][0], carto['t'], cy)   # aligned at the start of the eval window
            ax[0].plot(d['t'] - t0, np.degrees(wrap(y)), lw=0.8, color=col, label=lab + ' (offset removed)')
    ax[0].set_title('Yaw of all sources (odom-frame yaws aligned to cartographer at the window start)'); ax[0].set_ylabel('[deg]'); ax[0].set_ylim(-185, 185); ax[0].legend(loc='upper right')
    ax[0].set_xlabel('t [s]')
    finish(fig, P, 'Yaw', 'yaw.png')


def plot_timing(P, series):
    fig, ax = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    t0 = P['t0']
    cols = {'carto': COL['carto'], 'ekf': COL['ekf'], 'basalt': COL['basalt'], 'vesc': COL['vesc'], 'imu': COL['imu'],
            'tf map->odom': COL['alt'], 'tf odom->base': _eth['ETH_GRAY'], 'gt': COL['gt']}
    for lab, d in series.items():
        if d is None or len(d['t']) < 2:
            continue
        ax[0].plot(d['t'][1:] - t0, np.diff(d['t']) * 1e3, '.', ms=2, label=lab, color=cols.get(lab))
        ax[1].plot(d['t'] - t0, (d['t_recv'] - d['t']) * 1e3, '.', ms=2, label=lab, color=cols.get(lab))
    ax[0].set_title('Interval between consecutive header stamps'); ax[0].set_ylabel('[ms]'); ax[0].set_yscale('log'); ax[0].legend(ncol=3, fontsize=8)
    ax[1].set_title('Latency: record time minus header stamp'); ax[1].set_ylabel('[ms]'); ax[1].legend(ncol=3, fontsize=8); ax[1].set_xlabel('t [s]')
    finish(fig, P, 'Timing', 'timing.png')


# ----------------------------------------------------------------------------
def analyze(out_dir, bag=None, gt_topic=None, use_vio=True, gt_bag=None):
    out_dir = Path(out_dir)
    bag = Path(bag) if bag else out_dir / 'replay'
    P = {'plots': out_dir / 'plots', 'data': out_dir / 'data'}
    P['plots'].mkdir(parents=True, exist_ok=True)
    P['data'].mkdir(parents=True, exist_ok=True)

    topics = list(TOPICS.values()) + ([gt_topic] if gt_topic and not gt_bag else [])
    raw = read_bag(bag, topics)
    gt_twist_topic = gt_topic.replace('/pose', '/twist') if gt_topic and gt_topic.endswith('/pose') else None
    if gt_bag:
        raw.update(read_bag(gt_bag, [gt_topic] + ([gt_twist_topic] if gt_twist_topic else [])))
    elif gt_twist_topic:
        raw.update(read_bag(bag, [gt_twist_topic]))
    carto = poses(raw[TOPICS['carto']])
    ekf = poses(raw[TOPICS['ekf']])
    basalt = poses(raw[TOPICS['basalt']])
    vesc = poses(raw[TOPICS['vesc']])
    imu_d = imu(raw[TOPICS['imu']])
    map_odom = tf_pairs(raw[TOPICS['tf']], 'map', 'odom')
    odom_base = tf_pairs(raw[TOPICS['tf']], 'odom', 'base_link')
    gt = poses(raw[gt_topic]) if gt_topic else None
    gt_twist = twists(raw[gt_twist_topic]) if gt_twist_topic and raw.get(gt_twist_topic) else None
    missing = [k for k in EXPECTED if not raw[TOPICS[k]] and (use_vio or k != 'basalt')]
    if missing:
        alert('MISSING TOPICS in the replay bag: ' + ', '.join(f'{k} ({TOPICS[k]})' for k in missing)
              + '\n-> the corresponding node did not run or did not publish, check logs/launch.log. Stopping.')
        raise SystemExit(1)
    if gt is not None:
        # the mocap PC has its own clock: align GT stamps to the car by cross-correlating its yaw rate with the gyro
        gv = body_velocities(gt)
        off = time_offset(gv['t'], gv['vyaw'], imu_d['t'], imu_d['gz'], max_lag=5.0)
        if off is None:
            alert('ground truth: could not estimate the clock offset to the car (too short / no rotation)')
        else:
            gt['t'] = gt['t'] + off
            if gt_twist is not None:
                gt_twist['t'] = gt_twist['t'] + off
            print(f'ground truth clock offset to the car: {off*1e3:+.0f} ms (GT stamps shifted by it)', flush=True)
        overlap = min(gt['t'][-1], carto['t'][-1]) - max(gt['t'][0], carto['t'][0])
        if overlap < 5.0:
            alert(f'ground truth and bag overlap only {overlap:.1f} s in time: wrong GT bag, or clocks more than 5 s apart. Stopping.')
            raise SystemExit(1)
    P['t0'] = carto['t'][0]
    P['label'] = out_dir.resolve().name
    M = {'bag': str(bag), 'duration_s': float(carto['t'][-1] - carto['t'][0]), 'rates': {}, 'use_vio': use_vio}
    if gt is not None and off is not None:
        M['gt_clock_offset_s'] = off

    # timing plot on the whole bag, all numbers on the window after cartographer had time to settle
    plot_timing(P, {'carto': carto, 'ekf': ekf, 'basalt': basalt, 'vesc': vesc, 'imu': imu_d,
                    'tf map->odom': map_odom, 'tf odom->base': odom_base, 'gt': gt})

    full_v = body_velocities(carto)
    moving = np.hypot(full_v['vx'], full_v['vy']) > MOVE_SPEED
    t_move = full_v['t'][np.argmax(moving)] if moving.any() else carto['t'][0]
    t_start = t_move + WARMUP_S
    if map_odom is not None:
        # cartographer (re)localising: a jump of map->odom within the first INIT_S is still initialisation
        early = map_odom['t'][1:] < map_odom['t'][0] + INIT_S
        jump = (np.hypot(np.diff(map_odom['x']), np.diff(map_odom['y'])) > 0.5) | (np.abs(wrap(np.diff(map_odom['yaw']))) > math.radians(10))
        j = np.flatnonzero(early & jump)
        if len(j):
            t_start = max(t_start, map_odom['t'][j[-1] + 1] + WARMUP_S)
    M['window'] = {'t_first_motion_s': float(t_move - P['t0']), 'eval_start_s': float(t_start - P['t0']),
                   'eval_end_s': float(carto['t'][-1] - P['t0'])}
    carto, ekf, basalt, vesc, imu_d, map_odom, odom_base, gt, gt_twist = (
        crop(d, t_start) for d in (carto, ekf, basalt, vesc, imu_d, map_odom, odom_base, gt, gt_twist))
    if len(carto['t']) < 50:
        raise SystemExit(f'cartographer trajectory too short after the {WARMUP_S:g} s warm-up window')

    for name, d in (('carto', carto), ('ekf', ekf), ('basalt', basalt), ('vesc', vesc),
                    ('imu', imu_d), ('tf_map_odom', map_odom), ('tf_odom_base', odom_base), ('gt', gt)):
        if d is not None:
            M['rates'][name] = rate_stats(d)
            write_csv(P['data'] / f'{name}.csv', d)
    for name, d in (('carto', carto), ('ekf', ekf), ('basalt', basalt), ('gt', gt)):
        if d is not None:
            write_tum(P['data'] / f'{name}.tum', d)

    # --- global: cartographer ---------------------------------------------
    carto_v = body_velocities(carto)
    cv = {'t': carto['t'][1:]}
    dt = np.diff(carto['t'])
    v_at = np.interp(cv['t'], carto_v['t'], np.hypot(carto_v['vx'], carto_v['vy']))
    w_at = np.interp(cv['t'], carto_v['t'], carto_v['vyaw'])
    cv['pos'] = np.hypot(np.diff(carto['x']), np.diff(carto['y'])) - v_at * dt
    cv['yaw'] = wrap(np.diff(np.unwrap(carto['yaw']))) - w_at * dt
    M['carto_discontinuity'] = {'pos_m': stats(cv['pos']), 'yaw_rad': stats(cv['yaw']),
                                'pos_jumps_gt_5cm': int((np.abs(cv['pos']) > 0.05).sum()),
                                'yaw_jumps_gt_2deg': int((np.abs(cv['yaw']) > math.radians(2)).sum())}
    if map_odom is not None:
        dtm = np.diff(map_odom['t']); ok = dtm > 1e-4
        M['map_odom_correction'] = {
            'yaw_rate_deg_s': stats(np.abs(np.degrees(wrap(np.diff(np.unwrap(map_odom['yaw']))))[ok] / dtm[ok])),
            'pos_rate_m_s': stats(np.hypot(np.diff(map_odom['x']), np.diff(map_odom['y']))[ok] / dtm[ok]),
            'yaw_total_drift_deg': float(np.degrees(np.unwrap(map_odom['yaw'])[-1] - map_odom['yaw'][0])),
        }
        plot_global(P, map_odom)
    gt_ape = gt_v = None
    if gt is not None:
        gt_ape = evo_ape(gt, carto, GT_MAX_DT, align=True)
        M['carto_vs_gt_ape'] = {k: gt_ape[k]['stats'] for k in ('trans_m', 'rot_deg')}
        plot_rpe(P, gt_ape, 'Cartographer absolute pose error vs ground truth', 'carto_ape_gt.png', color=COL['carto'])
        gt_v = twist_body(gt_twist, gt) if gt_twist is not None else body_velocities(gt)
        if gt_v is not None and gt_v.get('gaps'):
            M['gt_tracking_gaps'] = gt_v['gaps']
            alert(f'ground truth has {gt_v["gaps"]} tracking gap(s): velocities masked there, check carto_ape_gt.pdf')
    plot_trajectories(P, carto, ekf, basalt, gt_ape, gt)
    ref_v = gt_v if gt_v is not None else carto_v
    write_csv(P['data'] / 'ref_velocity.csv', {k: ref_v[k] for k in ('t', 'vx', 'vy', 'vyaw')})

    # --- local: EKF ---------------------------------------------------------
    if ekf is not None:
        ref_name, ref = ('gt', gt) if gt is not None else ('cartographer', carto)
        rpe = evo_rpe(ref, ekf)
        M[f'ekf_rpe_{RPE_DELTA_S:g}s_vs_{ref_name}'] = {k: rpe[k]['stats'] for k in ('trans_m', 'rot_deg')}
        plot_rpe(P, rpe, f'EKF relative pose error over {RPE_DELTA_S:g} s vs {ref_name}', 'ekf_rpe.png')
        ve = {}
        for k in ('vx', 'vy', 'vyaw'):
            ve[k] = stats(ekf[k] - np.interp(ekf['t'], ref_v['t'], ref_v[k]))
        M[f'ekf_velocity_error_vs_{ref_name}'] = ve
        if odom_base is not None and map_odom is not None:
            chain = compose(map_odom, odom_base)
            chain_at = {k: interp_to(carto['t'], chain, k, angular=(k == 'yaw')) for k in ('x', 'y', 'yaw')}
            M['tf_chain_vs_tracked_pose'] = {
                'pos_m': stats(np.hypot(chain_at['x'] - carto['x'], chain_at['y'] - carto['y'])),
                'yaw_deg': stats(np.degrees(wrap(chain_at['yaw'] - carto['yaw'])))}
        if gt is not None:
            ape_gt_ekf = evo_ape(gt, ekf, GT_MAX_DT, align=True)
            M['ekf_vs_gt_ape_aligned'] = {k: ape_gt_ekf[k]['stats'] for k in ('trans_m', 'rot_deg')}
        plot_yaw(P, carto, ekf, basalt, imu_d)

    if gt is not None:
        M['laps'] = plot_laps(P, gt, 'gt', [('carto', carto), ('ekf', ekf), ('basalt', basalt)])
    else:
        M['laps'] = plot_laps(P, carto, 'carto', [('ekf', ekf), ('basalt', basalt)])

    # --- inputs -------------------------------------------------------------
    inp = {}
    for name, d, k in (('vesc_vx', vesc, 'vx'), ('basalt_vx', basalt, 'vx'), ('basalt_vy', basalt, 'vy'),
                       ('basalt_vyaw', basalt, 'vyaw'), ('imu_gz', imu_d, 'gz')):
        if d is not None:
            inp[name + '_vs_ref'] = stats(d[k] - np.interp(d['t'], ref_v['t'], ref_v[k if k != 'gz' else 'vyaw']))
    offs = {}
    if imu_d is not None:
        offs['gyro_vs_ref'] = time_offset(imu_d['t'], imu_d['gz'], ref_v['t'], ref_v['vyaw'])
        if basalt is not None:
            offs['basalt_vyaw_vs_gyro'] = time_offset(basalt['t'], basalt['vyaw'], imu_d['t'], imu_d['gz'])
    if basalt is not None:
        offs['basalt_vx_vs_ref'] = time_offset(basalt['t'], basalt['vx'], ref_v['t'], ref_v['vx'])
    if vesc is not None:
        offs['vesc_vx_vs_ref'] = time_offset(vesc['t'], vesc['vx'], ref_v['t'], ref_v['vx'])
    if ekf is not None:
        offs['ekf_vx_vs_ref'] = time_offset(ekf['t'], ekf['vx'], ref_v['t'], ref_v['vx'])
        offs['ekf_vyaw_vs_ref'] = time_offset(ekf['t'], ekf['vyaw'], ref_v['t'], ref_v['vyaw'])
    inp['time_offset_s'] = {k: v for k, v in offs.items() if v is not None}
    if imu_d is not None:
        still = np.hypot(np.interp(imu_d['t'], ref_v['t'], ref_v['vx']),
                         np.interp(imu_d['t'], ref_v['t'], ref_v['vy'])) < 0.05
        if still.sum() > 50:
            inp['imu_standstill_mean'] = {k: float(imu_d[k][still].mean()) for k in ('gx', 'gy', 'gz', 'ax', 'ay', 'az')}
    M['inputs'] = inp
    plot_velocities(P, ekf, vesc, basalt, imu_d, carto_v, gt_v)

    (P['data'] / 'metrics.json').write_text(json.dumps(M, indent=2))
    (out_dir / 'summary.md').write_text(summary(M))
    print(summary(M))
    return M


def summary(M):
    def f(s, k='rmse', fmt='{:.3f}'):
        return fmt.format(s[k]) if s and k in s else '-'

    R = M['rates']
    missing = [k for k in ('carto', 'ekf', 'basalt', 'vesc', 'imu') if k not in R and (M.get('use_vio', True) or k != 'basalt')]
    L = [f'# SE eval: {Path(M["bag"]).parent.name}/{Path(M["bag"]).name}  ({M["duration_s"]:.0f} s)', '']
    if missing:
        L.append('MISSING: ' + ', '.join(missing) + '  (not in the bag, check launch.log)')
    w = M.get('window')
    if w:
        L.append(f'eval window: {w["eval_start_s"]:.1f} .. {w["eval_end_s"]:.1f} s '
                 f'(first motion at {w["t_first_motion_s"]:.1f} s + {WARMUP_S:g} s warm-up skipped)')
    L += [
         'rates [Hz]: ' + ' | '.join(f'{k} {v["rate_hz"]:.0f}' for k, v in R.items()),
         'latency max [ms]: ' + ' | '.join(f'{k} {v["latency_max_s"]*1e3:.0f}' for k, v in R.items()),
         'non-monotonic stamps: ' + (', '.join(f'{k} {v["non_monotonic"]}' for k, v in R.items() if v['non_monotonic']) or 'none'),
         '']
    if 'map_odom_correction' in M:
        c = M['map_odom_correction']
        L.append(f'global: map->odom yaw drift {c["yaw_total_drift_deg"]:.1f} deg, correction rate median '
                 f'{f(c["yaw_rate_deg_s"], "median", "{:.1f}")} deg/s / {f(c["pos_rate_m_s"], "median", "{:.2f}")} m/s')
    d = M['carto_discontinuity']
    L.append(f'        tracked_pose jumps: {d["pos_jumps_gt_5cm"]} x >5 cm, {d["yaw_jumps_gt_2deg"]} x >2 deg')
    if 'carto_vs_gt_ape' in M:
        a = M['carto_vs_gt_ape']
        L.append(f'        APE vs GT rmse: {f(a["trans_m"])} m, {f(a["rot_deg"], fmt="{:.2f}")} deg')
    for k, v in M.items():
        if k.startswith('ekf_rpe'):
            L.append(f'local:  {k[8:]}: RPE rmse {f(v["trans_m"])} m / {f(v["rot_deg"], fmt="{:.2f}")} deg'
                     f' (max {f(v["trans_m"], "max")} m / {f(v["rot_deg"], "max", "{:.1f}")} deg)')
        if k.startswith('ekf_velocity_error'):
            L.append('        EKF velocity rmse: ' + ', '.join(f'{q} {f(v[q])}' for q in ('vx', 'vy', 'vyaw')))
    if 'tf_chain_vs_tracked_pose' in M:
        v = M['tf_chain_vs_tracked_pose']
        L.append(f'        TF chain vs tracked_pose rmse: {f(v["pos_m"])} m, {f(v["yaw_deg"], fmt="{:.2f}")} deg')
    I = M['inputs']
    if M.get('laps'):
        L.append(f'laps:   {len(M["laps"])} (plots/laps/), EKF APE per lap: ' + ' '.join(f'{l.get("ekf_ape_m", float("nan")):.3f}' for l in M['laps']) + ' m')
    L += ['', 'inputs vs reference, rmse: ' + ', '.join(f'{k[:-7]} {f(v)}' for k, v in I.items() if k.endswith('_vs_ref'))]
    if I.get('time_offset_s'):
        L.append('time offsets (+ = first signal early) [ms]: ' + ', '.join(f'{k} {v*1e3:+.0f}' for k, v in I['time_offset_s'].items()))
    if 'imu_standstill_mean' in I:
        L.append('imu standstill mean: ' + ' '.join(f'{k}={v:.3f}' for k, v in I['imu_standstill_mean'].items()))
    return '\n'.join(L) + '\n'


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out_dir', help='result folder (plots/, data/, summary.md go here)')
    ap.add_argument('--bag', help='bag to analyse (default: <out_dir>/replay)')
    ap.add_argument('--gt-topic', help='ground truth pose topic, if the bag has one')
    ap.add_argument('--gt-bag', help='separate bag holding the ground truth topic')
    a = ap.parse_args()
    meta = Path(a.out_dir) / 'config' / 'meta.json'   # defaults of a re-analysis: what the run was made with
    m = json.loads(meta.read_text()) if meta.exists() else {}
    analyze(a.out_dir, a.bag, a.gt_topic or m.get('gt_topic'), m.get('use_vio', m.get('run_vio', True)), a.gt_bag or m.get('gt_bag'))

