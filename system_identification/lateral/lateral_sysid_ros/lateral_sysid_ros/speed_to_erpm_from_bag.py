"""Estimates the VESC speed_to_erpm_gain from a bag: the localized speed against the motor erpm.

The offset stays as in vesc.yaml (or --offset); only the gain is fitted, on slow, steady,
nearly straight driving, where the wheels do not slip and the motor speed equals the
rear axle speed.
"""
import argparse

import matplotlib.pyplot as plt
import numpy as np
import yaml
from forza_config_ros import get_stack_master_path

from lateral_sysid_ros.steady_state_from_bag import read_bag

DT = 0.02  # [s] grid of the comparison
MIN_SAMPLES = 200


def yaw(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def window_mean(t_samples, values, t, window):
    """Mean of values over [t - window/2, t + window/2] for each t."""
    cumsum = np.concatenate(([0.0], np.cumsum(values)))
    i0, i1 = np.searchsorted(t_samples, t - window / 2), np.searchsorted(t_samples, t + window / 2)
    return (cumsum[i1] - cumsum[i0]) / np.maximum(i1 - i0, 1)


def fit_gain(v, erpm, offset, iterations=3):
    """Least-squares gain of erpm = gain * v + offset, rejecting outliers beyond 4 robust sigmas.

    Returns the gain and the mask of the samples it was fitted on.
    """
    keep = np.ones_like(v, dtype=bool)
    for _ in range(iterations):
        gain = np.sum(v[keep] * (erpm[keep] - offset)) / np.sum(v[keep] ** 2)
        residual = erpm - offset - gain * v
        mad = np.median(np.abs(residual[keep] - np.median(residual[keep])))
        keep &= np.abs(residual) <= 4 * 1.4826 * mad + 1e-9
    return gain, keep


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='rosbag2 folder')
    parser.add_argument('--car', required=True, help='racecar_version, e.g. NUC7')
    parser.add_argument('--pose-topic', default='/car_state/pose', help='PoseStamped of base_link (rear axle)')
    parser.add_argument('--core-topic', default='/sensors/core', help='VescStateStamped, state.speed in erpm')
    parser.add_argument('--offset', type=float, default=None,
                        help='speed_to_erpm_offset to keep fixed, default: the one in vesc.yaml')
    parser.add_argument('--window', type=float, default=1.0,
                        help='[s] the speed is the pose displacement over this window, the erpm its mean')
    parser.add_argument('--min-speed', type=float, default=0.5, help='[m/s]')
    parser.add_argument('--max-speed', type=float, default=1.5, help='[m/s] above this, wheel slip biases the gain')
    parser.add_argument('--max-acc', type=float, default=0.5, help='[m/s^2] steady driving only')
    parser.add_argument('--max-curvature', type=float, default=0.3,
                        help='[1/m] in tighter turns the motor speed is not the rear axle speed')
    parser.add_argument('--debug', action='store_true', help='plot the sample selection and the fit')
    args = parser.parse_args()

    config = get_stack_master_path(f'config/{args.car}')
    with open(config / 'vesc.yaml') as f:
        vesc = yaml.safe_load(f)
    general = vesc['/**']['ros__parameters']
    offset = general['speed_to_erpm_offset'] if args.offset is None else args.offset

    bag = read_bag(args.bag, [args.pose_topic, args.core_topic])
    t_pose, poses = bag[args.pose_topic]
    xy = np.array([[m.pose.position.x, m.pose.position.y] for m in poses])
    heading = np.unwrap([yaw(m.pose.orientation) for m in poses])
    t_core, cores = bag[args.core_topic]
    erpm_raw = np.array([m.state.speed for m in cores])

    half = args.window / 2
    t = np.arange(max(t_pose[0], t_core[0]) + half, min(t_pose[-1], t_core[-1]) - half, DT)
    displacement = np.stack([np.interp(t + half, t_pose, xy[:, i]) - np.interp(t - half, t_pose, xy[:, i])
                             for i in (0, 1)], axis=1) / args.window
    h = np.interp(t, t_pose, heading)
    v = displacement[:, 0] * np.cos(h) + displacement[:, 1] * np.sin(h)  # signed: reverse is negative
    yaw_rate = (np.interp(t + half, t_pose, heading) - np.interp(t - half, t_pose, heading)) / args.window
    erpm = window_mean(t_core, erpm_raw, t, args.window)
    acc = np.convolve(np.gradient(v, DT), np.ones(25) / 25, mode='same')

    candidates = ((np.abs(v) >= args.min_speed) & (np.abs(v) <= args.max_speed) & (np.abs(acc) <= args.max_acc)
                  & (np.abs(yaw_rate) <= args.max_curvature * np.abs(v)))
    if np.count_nonzero(candidates) < MIN_SAMPLES:
        raise SystemExit(f"Only {np.count_nonzero(candidates)} steady samples between {args.min_speed} and "
                         f"{args.max_speed} m/s in {args.bag}, need {MIN_SAMPLES}: drive longer, slowly and straight")
    gain, fitted = fit_gain(v[candidates], erpm[candidates], offset)
    used = np.zeros_like(candidates)
    used[np.flatnonzero(candidates)[fitted]] = True
    residual = erpm[used] - offset - gain * v[used]
    print(f"Fitted on {np.count_nonzero(used)} samples ({np.count_nonzero(used) * DT:.0f} s), speed "
          f"{np.abs(v[used]).min():.2f}-{np.abs(v[used]).max():.2f} m/s, residual std {np.std(residual):.0f} erpm")
    old_gain = general['speed_to_erpm_gain']
    print(f"speed_to_erpm_gain {gain:.1f} (vesc.yaml: {old_gain}, {100 * (gain / old_gain - 1):+.1f}%), "
          f"offset kept at {offset}")

    limits = vesc['actuation_manager']['ros__parameters']
    print(f"\nUpdate {config / 'vesc.yaml'}:\n"
          f"  speed_to_erpm_gain: {gain:.1f}\n"
          f"  speed_to_erpm_offset: {offset}\n"
          f"  actuation_manager speed_min / speed_max (now {limits['speed_min']} / {limits['speed_max']}) must stay within "
          f"{(general['speed_min'] - offset) / gain:.4f} / {(general['speed_max'] - offset) / gain:.4f}")

    if args.debug:
        fig, (ax_t, ax_v) = plt.subplots(1, 2, figsize=(14, 6))
        ax_t.plot(t - t[0], (erpm - offset) / old_gain, color='0.7', label=f'erpm with the old gain {old_gain}')
        ax_t.plot(t - t[0], (erpm - offset) / gain, color='k', label=f'erpm with the fitted gain {gain:.0f}')
        ax_t.plot(t - t[0], v, label='localized speed')
        ax_t.scatter(t[used] - t[0], v[used], s=4, color='C3', zorder=3, label='fitted samples')
        ax_t.set(xlabel='t [s]', ylabel='speed [m/s]', title='Sample selection')
        ax_v.scatter(v, erpm, s=2, color='0.8', label='all')
        ax_v.scatter(v[used], erpm[used], s=4, color='C3', label='fitted')
        v_line = np.array([v[used].min(), v[used].max()])
        ax_v.plot(v_line, gain * v_line + offset, 'k--', label=f'erpm = {gain:.0f} v + {offset}')
        ax_v.set(xlabel='localized speed [m/s]', ylabel='erpm', title='Fit')
        for ax in (ax_t, ax_v):
            ax.legend()
            ax.grid()
        plt.show()


if __name__ == '__main__':
    main()
