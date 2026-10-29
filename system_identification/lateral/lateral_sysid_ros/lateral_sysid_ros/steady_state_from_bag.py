"""Identifies the tires from bags of steady-state runs (e.g. recorded during steering_ramp_node)
and writes tires + steering LUT to stack_master/config."""
import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from on_track_sysid import identify_steady_state
from on_track_sysid.pacejka import steady_state_forces
from forza_config_ros import load_car_config_ros
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from lateral_sysid_ros.config_io import data_dir, load_tires, load_vehicle, plot_tires, save_results


def read_bag(uri, topics):
    """Returns {topic: (stamps [s], messages)} for the given topics."""
    with open(Path(uri) / 'metadata.yaml') as f:
        storage_id = yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    if missing:
        raise ValueError(f"{uri} has no topics {missing}; it has {sorted(types)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))

    out = {t: ([], []) for t in topics}
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        out[topic][0].append(stamp * 1e-9)
        out[topic][1].append(deserialize_message(raw, get_message(types[topic])))
    return {t: (np.array(s), m) for t, (s, m) in out.items()}


def check_imu_frame(frame_ids, a_x, a_y, a_z, v_x, omega, expected_frame='base_link', min_samples=100):
    """Checks that the IMU is in the car frame (x forward, y left, z up).

    Returns an error message, or None when the IMU looks right. Besides the frame_id it
    checks the physics: while cornering a_y follows v_x * omega, and a_z holds gravity.
    """
    frames = {f.split('/')[-1] for f in frame_ids}
    if frames != {expected_frame}:
        return f"IMU frame_id is {sorted(frames)}, expected '{expected_frame}'"
    if abs(np.median(a_z) - 9.81) > 2.0:
        return f"IMU median a_z is {np.median(a_z):.2f} m/s^2, expected about +9.81 (z up, gravity included)"
    a_lat = v_x * omega
    cornering = np.abs(a_lat) > 1.0
    if np.count_nonzero(cornering) < min_samples:
        print(f"Warning: only {np.count_nonzero(cornering)} cornering samples, cannot check the IMU axes")
        return None
    corr_y = np.corrcoef(a_y[cornering], a_lat[cornering])[0, 1]
    corr_x = np.nan_to_num(np.corrcoef(a_x[cornering], a_lat[cornering])[0, 1])  # constant a_x gives nan
    if not corr_y >= 0.5 or abs(corr_x) > corr_y:  # also rejects a constant a_y (nan)
        return (f"IMU axes do not match the car frame: correlation with v_x * omega is {corr_y:.2f} for a_y "
                f"and {corr_x:.2f} for a_x (expected a_y close to 1)")
    return None


def bag_to_data(uri, args, C_0d, C_d):
    """Samples v_x, v_y, omega, delta (and a_x) at the odometry stamps; also returns their median period."""
    steer_topic = args.servo_topic or args.steer_topic
    topics = [args.odom_topic, steer_topic] + ([args.imu_topic] if args.imu_topic else [])
    bag = read_bag(uri, topics)

    t, odom = bag[args.odom_topic]
    data = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z, 0.0] for m in odom])

    t_steer, steer_msgs = bag[steer_topic]
    if args.servo_topic:
        steer = (np.array([m.data for m in steer_msgs]) - C_0d) / C_d
    else:
        steer = np.array([m.drive.steering_angle for m in steer_msgs])
    # hold the last steering command; drop odometry from before the first one
    idx = np.searchsorted(t_steer, t, side='right') - 1
    keep = idx >= 0
    data, t, idx = data[keep], t[keep], idx[keep]
    data[:, 3] = steer[idx]

    a_x = None
    if args.imu_topic:
        t_imu, imu = bag[args.imu_topic]
        acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in imu])
        a_x, a_y, a_z = (np.interp(t, t_imu, acc[:, i]) for i in range(3))
        error = check_imu_frame([m.header.frame_id for m in imu], a_x, a_y, a_z, data[:, 0], data[:, 2],
                                args.imu_frame)
        if error:
            raise SystemExit(f"{uri}: {error}. Use an IMU topic in the car frame, or --imu-topic '' "
                             "to use static axle loads.")
    return data, a_x, np.median(np.diff(t))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='+', help='rosbag2 folders')
    parser.add_argument('--car', required=True, help='racecar_version, e.g. NUC6')
    parser.add_argument('--floor', required=True)
    parser.add_argument('--odom-topic', default='/state_estimation/odom')
    parser.add_argument('--steer-topic', default='/drive', help='AckermannDriveStamped steering commands')
    parser.add_argument('--servo-topic', default='',
                        help='Float64 servo commands (e.g. /vesc/commands/servo/position); used instead of '
                             '--steer-topic when set, converted with C_0d and C_d of car_model.yaml')
    parser.add_argument('--imu-topic', default='/imu',
                        help="IMU in the car frame, for load transfer; '' uses static axle loads")
    parser.add_argument('--imu-frame', default='base_link', help='expected frame_id of the IMU messages')
    parser.add_argument('--vy-at-cog', action='store_true', help='odometry v_y is at the center of gravity '
                        '(e.g. in simulation) instead of the rear axle')
    parser.add_argument('--equal-tires', action='store_true')
    parser.add_argument('--no-plot', action='store_true')
    parser.add_argument('--no-update-latest', action='store_true')
    args = parser.parse_args()

    car = load_car_config_ros(args.car)
    vehicle = load_vehicle(args.car)
    init = load_tires(args.car, args.floor)

    runs = [bag_to_data(uri, args, car.C_0d, car.C_d) for uri in args.bags]
    data = np.concatenate([d for d, _, _ in runs])
    a_x = None if not args.imu_topic else np.concatenate([a for _, a, _ in runs])
    dt = float(np.median([p for _, _, p in runs]))
    print(f"Loaded {len(data)} samples from {len(args.bags)} bag(s)")

    tires = identify_steady_state(data, vehicle, init, a_x=a_x, vy_at_rear_axle=not args.vy_at_cog,
                                  equal_tires=args.equal_tires, dt=dt, log=print)
    print(f"Identified front {np.round(tires.front, 4)}, rear {np.round(tires.rear, 4)}")
    if not args.no_plot:
        v_x, v_y, omega, delta = data[data[:, 0] > 1.0].T
        if not args.vy_at_cog:
            v_y = v_y + vehicle.lr * omega
        alpha_f, alpha_r, _, _, F_yf, F_yr = steady_state_forces(vehicle, v_x, v_y, omega, delta)
        path = data_dir(args.car, args.floor) / 'steady_state_fit.png'
        plot_tires(path, vehicle, tires, init, (alpha_f, F_yf, alpha_r, F_yr))
        print(f"Saved fit plot to {path}")
    save_results(tires, vehicle, args.car, args.floor, not args.no_update_latest)
