"""Estimates the maximum steering angle from a bag driving full lock, first one way then the other."""
import argparse

import matplotlib.pyplot as plt
import numpy as np
import yaml
from forza_config_ros import get_stack_master_path

from lateral_sysid_ros.config_io import load_vehicle
from lateral_sysid_ros.steady_state_from_bag import read_bag

ASYMMETRY_TOL = 0.05  # relative radius difference that hints at an off-center servo
MAX_LAT_ACC = 1.0  # [m/s^2] above this, tire slip widens the circle and the angle comes out low
STEADY_TOL = 0.1  # relative deviation of the smoothed yaw rate from its median that counts as steady state


def fit_circle(x, y):
    """Least-squares (Kasa) circle fit, returns center x, center y, radius."""
    a, b, c = np.linalg.lstsq(np.c_[x, y, np.ones_like(x)], x**2 + y**2, rcond=None)[0]
    return a / 2, b / 2, np.sqrt(c + (a**2 + b**2) / 4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='rosbag2 folder')
    parser.add_argument('--car', required=True, help='racecar_version, e.g. NUC6')
    parser.add_argument('--pose-topic', default='/car_state/pose', help='PoseStamped of base_link (rear axle)')
    parser.add_argument('--imu-topic', default='/imu', help='IMU in base_link, for the yaw rate')
    parser.add_argument('--debug', action='store_true', help='plot the steady-state selection and circle fits')
    args = parser.parse_args()

    bag = read_bag(args.bag, [args.pose_topic, args.imu_topic])
    t, poses = bag[args.pose_topic]
    x, y = np.array([[m.pose.position.x, m.pose.position.y] for m in poses]).T
    t_imu, imu = bag[args.imu_topic]
    yaw_rate_raw = np.array([m.angular_velocity.z for m in imu])
    yaw_rate_imu = np.convolve(yaw_rate_raw, np.ones(25) / 25, mode='same')  # 0.5 s at 50 Hz
    yaw_rate = np.interp(t, t_imu, yaw_rate_imu)

    L = load_vehicle(args.car).wheelbase
    fig, (ax_w, ax_xy) = plt.subplots(1, 2, figsize=(14, 6))
    ax_w.plot(t_imu - t[0], yaw_rate_raw, color='0.8', label='IMU yaw rate')
    ax_w.plot(t_imu - t[0], yaw_rate_imu, color='k', label='smoothed')
    ax_xy.plot(x, y, color='0.8', label='pose')

    radii = {}
    for side, sign in (('left', 1), ('right', -1)):
        turning = sign * yaw_rate > 0.3
        if np.count_nonzero(turning) < 10:
            raise SystemExit(f"Not enough {side} turning samples in {args.bag}")
        w_ss = np.median(yaw_rate[turning])
        steady = turning & (np.abs(yaw_rate - w_ss) < STEADY_TOL * abs(w_ss))
        cx, cy, radii[side] = fit_circle(x[steady], y[steady])
        a_lat = radii[side] * w_ss**2
        print(f"{side}: radius {radii[side]:.3f} m, steering {np.arctan(L / radii[side]):.4f} rad, "
              f"yaw rate {w_ss:.3f} rad/s, lateral acceleration {a_lat:.2f} m/s^2, {np.count_nonzero(steady)} samples")
        if a_lat > MAX_LAT_ACC:
            print(f"WARNING: {side} lateral acceleration {a_lat:.2f} m/s^2 > {MAX_LAT_ACC} m/s^2, "
                  "drive slower to stay kinematic")

        ax_w.scatter(t[steady] - t[0], yaw_rate[steady], s=4, zorder=3, label=f'{side} steady state')
        ax_w.axhspan(w_ss * (1 - STEADY_TOL), w_ss * (1 + STEADY_TOL), alpha=0.1)
        ax_xy.scatter(x[steady], y[steady], s=4, label=f'{side} steady state')
        phi = np.linspace(0, 2 * np.pi, 200)
        ax_xy.plot(cx + radii[side] * np.cos(phi), cy + radii[side] * np.sin(phi), '--',
                   label=f'{side} fit R={radii[side]:.3f} m')

    if abs(radii['left'] - radii['right']) / min(radii.values()) > ASYMMETRY_TOL:
        print(f"WARNING: left and right radii differ by more than {ASYMMETRY_TOL:.0%}, "
              "the servo may need mechanical centering")
    R = np.mean(list(radii.values()))
    print(f"Mean radius {R:.3f} m, max steering angle {np.arctan(L / R):.4f} rad")

    ax_w.set(xlabel='t [s]', ylabel='yaw rate [rad/s]', title='Steady-state selection')
    ax_xy.set(xlabel='x [m]', ylabel='y [m]', title='Circle fits', aspect='equal')
    for ax in (ax_w, ax_xy):
        ax.legend()
        ax.grid()
    if args.debug:
        plt.show()

    config = get_stack_master_path(f'config/{args.car}')
    with open(config / 'vesc.yaml') as f:
        vesc = yaml.safe_load(f)['/**']['ros__parameters']
    # servo_min/servo_max are full lock; the sign of the current gain tells which one is left
    servo_left, servo_right = ((vesc['servo_max'], vesc['servo_min']) if vesc['steering_angle_to_servo_gain'] > 0
                               else (vesc['servo_min'], vesc['servo_max']))
    delta_left, delta_right = np.arctan(L / radii['left']), np.arctan(L / radii['right'])
    gain = (servo_left - servo_right) / (delta_left + delta_right)
    print(f"\nUpdate {config / 'car_model.yaml'}:\n"
          f"  max_steering_angle: {np.arctan(L / R):.4f}\n"
          f"  min_radius_curv_m: {R:.3f}\n"
          f"Update {config / 'vesc.yaml'} (servo mapping from servo_min/servo_max = full lock):\n"
          f"  steering_angle_to_servo_gain: {gain:.4f}\n"
          f"  steering_angle_to_servo_offset: {servo_left - gain * delta_left:.4f}\n"
          f"  steering_min: {-delta_right:.4f}  # actuation_manager\n"
          f"  steering_max: {delta_left:.4f}  # actuation_manager\n"
          f"Update every {config / '*_mpc_params.yaml'} to match vesc.yaml:\n"
          f"  delta_min: {-delta_right:.4f}\n"
          f"  delta_max: {delta_left:.4f}\n"
          f"Update human_control drive-steering_angle in {config / 'joy_teleop.yaml'} so full stick reaches "
          f"full lock on both sides:\n"
          f"  scale: {max(delta_left, delta_right):.4f}")


if __name__ == '__main__':
    main()
