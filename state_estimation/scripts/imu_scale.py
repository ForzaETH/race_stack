#!/usr/bin/env python3
"""Gyro scale factor of the VESC IMU from a driven bag and a yaw reference.

  python3 state_estimation/scripts/imu_scale.py bags/<bag> [--gt-bag bags/<gt>] [--car NUC6]

gyro_scale = net yaw of the reference / integrated (bias-corrected) raw gyro yaw over the same time.
Reference: `/<car>/pose` (mocap) from --gt-bag or the bag itself, else `/tracked_pose` (cartographer, localised
on a good map). Net yaw is frame independent, so the reference needs no alignment. Drive whole laps, the more
turned angle the better: at 10 revolutions a 0.5 % error is 18 deg, far above the reference noise.
The bias from stack_master/config/<car>/imu.yaml is subtracted first (it is the same order of magnitude as the
scale error over a long bag), the raw gyro is rotated into base_link with the IMU mount of static_transforms.yaml.
No ROS needed (rosbags).
"""

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'eval'))   # bag readers of analyze.py
from analyze import imu, poses, read_bag, time_offset  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
trapz = getattr(np, 'trapezoid', None) or np.trapz
HALVES_TOL = 0.005   # the two halves of a bag should give the same scale within this


def mount_rotation(car, child):
    """base_link <- IMU rotation matrix from stack_master/config/<car>/static_transforms.yaml."""
    from scipy.spatial.transform import Rotation
    cfg = yaml.safe_load((REPO / f'stack_master/config/{car}/static_transforms.yaml').read_text())
    for tr in cfg['transforms']:
        if tr['child'] == child:
            return (Rotation.from_quat(tr['quat']) if 'quat' in tr else Rotation.from_euler('xyz', tr['rpy'])).as_matrix()
    raise SystemExit(f'no transform base_link -> {child} in static_transforms.yaml of {car}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='bag of a drive with /sensors/imu/raw (and /tracked_pose unless --gt-bag)')
    ap.add_argument('--gt-bag', help='bag with the mocap pose topic /<car>/pose')
    ap.add_argument('--car', default=os.environ.get('RACECAR_VERSION'), help='NUCx (default: $RACECAR_VERSION)')
    ap.add_argument('--topic', default='/sensors/imu/raw', help='raw IMU topic')
    ap.add_argument('--no-bias', action='store_true', help='do not subtract the gyro_bias of imu.yaml')
    a = ap.parse_args()
    if not a.car:
        ap.error('--car is required when RACECAR_VERSION is not set')

    raw = read_bag(a.bag, [a.topic, '/tracked_pose', f'/{a.car.lower()}/pose'])
    gt_topic = f'/{a.car.lower()}/pose'
    if a.gt_bag:
        raw.update(read_bag(a.gt_bag, [gt_topic]))
    d = imu(raw[a.topic])
    if d is None:
        raise SystemExit(f'no {a.topic} in {a.bag}')
    ref_name, ref = (gt_topic, poses(raw[gt_topic])) if raw[gt_topic] else ('/tracked_pose', poses(raw['/tracked_pose']))
    if ref is None:
        raise SystemExit(f'no yaw reference: neither {gt_topic} nor /tracked_pose')

    imu_cfg = yaml.safe_load((REPO / f'stack_master/config/{a.car}/imu.yaml').read_text())['imu_to_base']['ros__parameters']
    bias = np.zeros(3) if a.no_bias else np.array(imu_cfg.get('gyro_bias', [0, 0, 0]), float)
    R = mount_rotation(a.car, raw[a.topic][0][1].header.frame_id)
    wz = (R @ (np.c_[d['gx'], d['gy'], d['gz']] - bias).T)[2]          # base_link yaw rate, unscaled

    yaw = np.unwrap(ref['yaw'])
    ref_rate = np.gradient(yaw, ref['t'])
    off = time_offset(ref['t'], ref_rate, d['t'], wz, max_lag=5.0) or 0.0   # > 0: reference stamps early, car time = ref time + off
    t0, t1 = ref['t'][0] + off, ref['t'][-1] + off
    m = (d['t'] >= t0) & (d['t'] <= t1)
    if m.sum() < 100:
        raise SystemExit('reference and IMU do not overlap in time')
    gyro_yaw = trapz(wz[m], d['t'][m])
    ref_yaw = yaw[-1] - yaw[0]
    scale = ref_yaw / gyro_yaw
    # consistency: same ratio on the two halves of the bag
    tm = 0.5 * (t0 + t1)
    halves = []
    for lo, hi in ((t0, tm), (tm, t1)):
        mm = (d['t'] >= lo) & (d['t'] <= hi)
        r_lo, r_hi = np.interp([lo - off, hi - off], ref['t'], yaw)
        halves.append((r_hi - r_lo) / trapz(wz[mm], d['t'][mm]))

    print(f'reference {ref_name}: {len(ref["t"])} poses over {t1 - t0:.1f} s, clock offset to the car {off * 1e3:+.0f} ms')
    print(f'bias subtracted [rad/s]: {bias.tolist()}   mount: {raw[a.topic][0][1].header.frame_id} -> base_link')
    print(f'net yaw: reference {math.degrees(ref_yaw):+.1f} deg ({ref_yaw / (2 * math.pi):+.2f} rev), '
          f'gyro {math.degrees(gyro_yaw):+.1f} deg, difference {math.degrees(ref_yaw - gyro_yaw):+.2f} deg')
    print(f'scale: {scale:.4f}  (first half {halves[0]:.4f}, second half {halves[1]:.4f}; current imu.yaml {imu_cfg.get("gyro_scale", 1.0)})')
    if abs(ref_yaw) < 4 * math.pi:
        print('WARNING: less than 2 revolutions of net yaw, the estimate is noise dominated; drive whole laps')
    if abs(halves[0] - halves[1]) > HALVES_TOL:
        print(f'WARNING: the two halves disagree by more than {HALVES_TOL * 100:g} %: too little turning, a jumping reference, or a wrong bias')
    print(f'\nstack_master/config/{a.car}/imu.yaml:')
    print(f'    gyro_scale: {scale:.4f}  # {Path(a.bag).name} vs {ref_name}, {ref_yaw / (2 * math.pi):+.1f} rev')


if __name__ == '__main__':
    main()
