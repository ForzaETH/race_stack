#!/usr/bin/env python3
"""Latency and loss of same-host pub/sub over zenoh, with shared memory off and on.

Publishes UInt8MultiArray messages of several sizes from one process to another (sensor QoS) and
reports the latency (publish call -> subscriber callback, includes (de)serialization) and the loss.
Every case runs once per mode; the mode only applies to the two test processes, so the running stack
is left alone. A router is started for the test when none is listening on port 7447.

Run inside the ROS container:
  python3 zenoh_shm_eval.py [--seconds 8] [--modes off on]
"""

import argparse
import array
import json
import os
import socket
import struct
import subprocess
import sys
import time

# (payload bytes, rate Hz): odometry-sized, a laser scan, a small image, three sizes of large image / cloud
CASES = [(256, 100), (8192, 40), (131072, 30), (1048576, 30), (2097152, 30), (8388608, 15)]
TOPIC = 'zenoh_shm_eval'
SHM_KEY = 'transport/shared_memory/enabled='
WARMUP = 5      # first messages are dropped from the statistics (discovery, lazy SHM segment allocation)


def publish(size, hz, seconds):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import UInt8MultiArray
    rclpy.init()
    node = rclpy.create_node('zenoh_shm_eval_pub')
    pub = node.create_publisher(UInt8MultiArray, TOPIC, qos_profile_sensor_data)
    msg = UInt8MultiArray()
    buf = array.array('B', bytes(size))
    time.sleep(2.0)     # discovery
    seq, end = 0, time.monotonic() + seconds
    while time.monotonic() < end:
        struct.pack_into('<qq', buf, 0, seq, time.monotonic_ns())
        msg.data = buf
        pub.publish(msg)
        seq += 1
        time.sleep(1.0 / hz)


def subscribe(size, seconds):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import UInt8MultiArray
    rclpy.init()
    node = rclpy.create_node('zenoh_shm_eval_sub')
    lat, seqs = [], []

    def callback(msg):
        now = time.monotonic_ns()
        seq, sent = struct.unpack_from('<qq', msg.data, 0)
        lat.append((now - sent) / 1e6)
        seqs.append(seq)

    node.create_subscription(UInt8MultiArray, TOPIC, callback, qos_profile_sensor_data)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    lat = sorted(lat[WARMUP:])
    sent = seqs[-1] - seqs[0] + 1 if seqs else 0

    def quantile(p):
        return lat[min(len(lat) - 1, int(p * len(lat)))] if lat else float('nan')

    print(json.dumps({'size': size, 'received': len(seqs), 'lost_pct': 100.0 * (sent - len(seqs)) / sent if sent else 100.0,
                      'p50': quantile(0.5), 'p99': quantile(0.99), 'max': lat[-1] if lat else float('nan')}))


def router_running():
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex(('127.0.0.1', 7447)) == 0


def shm_segments():
    return len(os.listdir('/dev/shm'))


def run_case(size, hz, seconds, env):
    me = [sys.executable, os.path.abspath(__file__)]
    before = shm_segments()
    sub = subprocess.Popen(me + ['sub', str(size), str(hz), str(seconds + 4)], env=env,
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    time.sleep(1.0)
    pub = subprocess.Popen(me + ['pub', str(size), str(hz), str(seconds)], env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0 + seconds / 2)
    segments = shm_segments() - before
    out = sub.communicate()[0]
    pub.wait()
    result = json.loads(out.strip().splitlines()[-1])
    result.update(hz=hz, segments=segments)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=8.0, help='publishing time per case (default 8)')
    ap.add_argument('--modes', nargs='+', choices=['off', 'on'], default=['off', 'on'])
    a = ap.parse_args()

    results = {}
    for mode in a.modes:
        env = dict(os.environ, RMW_IMPLEMENTATION='rmw_zenoh_cpp',
                   ZENOH_CONFIG_OVERRIDE=SHM_KEY + ('true' if mode == 'on' else 'false'))
        router = None
        if not router_running():
            router = subprocess.Popen(['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'], env=env,
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
            time.sleep(2.0)
        try:
            for size, hz in CASES:
                r = results[mode, size] = run_case(size, hz, a.seconds, env)
                print(f'shm {mode:3s} {size:>8d} B @ {hz:3d} Hz: p50 {r["p50"]:6.2f}  p99 {r["p99"]:6.2f}  max {r["max"]:6.2f} ms, '
                      f'lost {r["lost_pct"]:.1f} %, {r["received"]} msgs, {r["segments"]:+d} /dev/shm entries', flush=True)
        finally:
            if router:
                os.killpg(router.pid, 15)
                router.wait()

    if len(a.modes) == 2:
        print('\n   size [B]    Hz |  p50 off -> on [ms]  |  p99 off -> on [ms]  | lost off / on [%]')
        for size, hz in CASES:
            off, on = results['off', size], results['on', size]
            print(f'{size:>11d} {hz:5d} | {off["p50"]:7.2f} -> {on["p50"]:7.2f}   | {off["p99"]:7.2f} -> {on["p99"]:7.2f}   '
                  f'| {off["lost_pct"]:5.1f} / {on["lost_pct"]:.1f}')
    if 'on' in a.modes:
        active = any(results['on', size]['segments'] > 0 for size, _ in CASES)
        print('\nshared memory ' + ('ACTIVE: new /dev/shm segments while publishing with it on' if active else
              'NOT ACTIVE: no new /dev/shm segments with it on (rmw_zenoh too old, or /dev/shm full?)'))


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'pub':
        publish(int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    elif len(sys.argv) > 1 and sys.argv[1] == 'sub':
        subscribe(int(sys.argv[2]), float(sys.argv[4]))
    else:
        main()
