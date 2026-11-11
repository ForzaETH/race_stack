#!/usr/bin/env python3
"""Gyro bias of the VESC IMU from a bag recorded at standstill.

  python3 state_estimation/scripts/imu_bias.py bags/standstill

Prints the mean angular velocity per axis in the IMU's own frame, i.e. the `gyro_bias` line for
stack_master/config/<car>/imu.yaml (imu_to_base_node subtracts it before rotating into base_link).
Record 30-60 s with the car on the ground, motor off, nobody touching it; the script refuses a bag with motion.
No ROS needed (rosbags).
"""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'eval'))   # bag readers of analyze.py
from analyze import imu, read_bag  # noqa: E402

MAX_GYRO_STD = 0.02    # rad/s, above this the car was not at rest (sensor noise is ~0.005)
MAX_ACCEL_STD = 0.3    # m/s^2
MIN_S = 20.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='bag recorded at standstill')
    ap.add_argument('--topic', default='/sensors/imu/raw', help='raw IMU topic (default: /sensors/imu/raw)')
    a = ap.parse_args()

    d = imu(read_bag(a.bag, [a.topic])[a.topic])
    if d is None:
        raise SystemExit(f'no {a.topic} in {a.bag}')
    dur = d['t'][-1] - d['t'][0]
    g = np.c_[d['gx'], d['gy'], d['gz']]
    acc = np.c_[d['ax'], d['ay'], d['az']]
    print(f'{len(d["t"])} samples, {dur:.1f} s, {len(d["t"]) / dur:.0f} Hz')
    print('gyro  mean [rad/s]: ' + '  '.join(f'{v:+.5f}' for v in g.mean(0)) + '   std: ' + '  '.join(f'{v:.4f}' for v in g.std(0)))
    print('accel mean [m/s^2]: ' + '  '.join(f'{v:+.3f}' for v in acc.mean(0)) + f'   |a| {np.linalg.norm(acc.mean(0)):.3f}'
          + '   std: ' + '  '.join(f'{v:.3f}' for v in acc.std(0)))
    bad = []
    if dur < MIN_S:
        bad.append(f'only {dur:.0f} s, record at least {MIN_S:.0f} s')
    if (g.std(0) > MAX_GYRO_STD).any():
        bad.append(f'gyro std above {MAX_GYRO_STD} rad/s: the car moved')
    if (acc.std(0) > MAX_ACCEL_STD).any():
        bad.append(f'accel std above {MAX_ACCEL_STD} m/s^2: the car moved or vibrated (motor on?)')
    if bad:
        raise SystemExit('not usable: ' + '; '.join(bad))
    print('\nstack_master/config/<car>/imu.yaml:')
    print('    gyro_bias: [' + ', '.join(f'{v:.4f}' for v in g.mean(0)) + f']  # measured on {Path(a.bag).name}, {dur:.0f} s at rest')


if __name__ == '__main__':
    main()
