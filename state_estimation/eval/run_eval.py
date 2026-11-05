#!/usr/bin/env python3
"""Headless state estimation evaluation on a recorded bag.

1. starts state_estimation_launch.xml on sim time + vesc_to_odom_node (recomputes /odom from the raw VESC state)
   + static_transforms_launch.xml (mounts from static_transforms.yaml; the RealSense-internal TF tree comes from the bag)
2. records the estimator outputs + raw inputs into <out>/replay
3. plays the raw sensor topics of the bag (+ the ground truth topic if the bag has it), waits until it is done
4. stops everything, copies the SLAM / EKF / VIO / TF / sensor configs into <out>/config, process logs are in <out>/logs
5. runs analyze.py -> <out>/plots, <out>/data (csv, tum, metrics.json), <out>/summary.md
6. points results/latest to <out>

Run inside the ROS container:
  USE_VIO=true python3 state_estimation/eval/run_eval.py bags/<bag> --gt-bag bags/<gt> --map <map> [--car NUC6] [--name x]
USE_VIO=true runs Basalt on the bagged images, USE_VIO=false runs the EKF on VESC + IMU only.
Re-run only the analysis: python3 analyze.py results/<run>
"""

import argparse
import datetime
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
from analyze import analyze  # noqa: E402

# raw sensor topics that get played; /odom is recomputed by vesc_to_odom_node from the VESC state, /tf_static is published
# from static_transforms.yaml (+ the camera_* frames of the bag), so the mounts can be tuned offline
PLAY = ['/scan', '/sensors/imu/raw', '/sensors/core', '/sensors/servo_position_command']
PLAY_VIO = ['/camera/camera/imu', '/camera/camera/infra1/image_rect_raw', '/camera/camera/infra2/image_rect_raw']
# what gets recorded for the analysis
RECORD = ['/tf', '/tf_static', '/tracked_pose', '/state_estimation/odom', '/basalt/odom', '/basalt/odomimu',
          '/odom', '/imu']
SIZE_WARN_GB = 1.0
RESULTS = HERE / 'results'
SETTLE_S = 6.0          # time for the nodes (cartographer pbstream load) before recording / playing
GT_TOPIC = '/{car}/pose'  # ground truth pose topic (Vicon bridge, car name lower-case); used when the bag has it


def bag_meta(bag):
    return yaml.safe_load((Path(bag) / 'metadata.yaml').read_text())['rosbag2_bagfile_information']


def bag_duration(bag):
    return bag_meta(bag)['duration']['nanoseconds'] * 1e-9


def bag_topics(bag):
    return {t['topic_metadata']['name'] for t in bag_meta(bag)['topics_with_message_count']}


def run(cmd, **kw):
    print('$', ' '.join(cmd), flush=True)
    return subprocess.Popen(cmd, start_new_session=True, **kw)


def stop(p, timeout=15):
    if p is None or p.poll() is not None:
        return
    os.killpg(os.getpgid(p.pid), signal.SIGINT)
    try:
        p.wait(timeout)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(p.pid), signal.SIGKILL)


def fail(msg, procs=()):
    bar = '!' * 78
    print(f'\033[1;91m{bar}\n{msg}\n{bar}\033[0m', flush=True)
    for p in procs:
        stop(p)
    raise SystemExit(1)


def check_nodes(out, procs, required, timeout=20.0):
    """Wait until all required nodes are up (composable nodes load late), fail on death or timeout."""
    deadline = time.time() + timeout
    while True:
        dead = [l.strip() for l in (out / 'logs' / 'launch.log').read_text().splitlines() if 'process has died' in l]
        if dead:
            fail('a node died during startup:\n' + '\n'.join(d[:160] for d in dead) + f'\nsee {out / "logs" / "launch.log"}', procs)
        nodes = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True).stdout
        missing = [n for n in required if n not in nodes]
        if not missing:
            return
        if time.time() > deadline:
            fail('required nodes not running: ' + ', '.join(missing) + f'\nrunning: {" ".join(nodes.split())}'
                 + f'\nsee {out / "logs" / "launch.log"}', procs)
        time.sleep(1.0)


def check_publishers(out, procs, topics):
    missing = []
    for t in topics:
        info = subprocess.run(['ros2', 'topic', 'info', t], capture_output=True, text=True).stdout
        if 'Publisher count: 0' in info or not info:
            missing.append(t)
    if missing:
        fail('no publisher on ' + ', '.join(missing) + ' while the bag is playing\n'
             + f'-> the node is up but silent (TF lookup, calibration, topic names?), see {out / "logs" / "launch.log"}', procs)


def dir_size(path):
    return sum(f.stat().st_size for f in Path(path).rglob('*') if f.is_file())


def warn_size(results):
    total = dir_size(results)
    print(f'results folder: {total / 1e9:.2f} GB in {results}')
    if total > SIZE_WARN_GB * 1e9:
        runs = sorted(((dir_size(d), d.name) for d in results.iterdir() if d.is_dir() and not d.is_symlink()), reverse=True)
        print(f'WARNING: results exceed {SIZE_WARN_GB:g} GB. Largest runs:')
        for sz, name in runs[:5]:
            print(f'  {sz / 1e6:7.0f} MB  {name}')
        print('delete old runs or their replay/ bags (plots, metrics and data/ are enough to look back at).')


def bag_static_tf(bag):
    """All static transforms of the bag as a list of dicts (same layout as static_transforms.yaml)."""
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    found = {}
    with Reader(bag) as reader:
        conns = [c for c in reader.connections if c.topic == '/tf_static']
        for conn, _, raw in reader.messages(connections=conns):
            for tr in typestore.deserialize_cdr(raw, conn.msgtype).transforms:
                t, q = tr.transform.translation, tr.transform.rotation
                found[(tr.header.frame_id, tr.child_frame_id)] = {
                    'parent': tr.header.frame_id, 'child': tr.child_frame_id,
                    'xyz': [t.x, t.y, t.z], 'quat': [q.x, q.y, q.z, q.w]}
    return list(found.values())


def save_configs(out, car, bag):
    """Copy the configs the run was made with into <out>/config/{ekf,slam,vio,tf,sensors}.
    tf/: static_transforms.yaml (what the run used) and the bag's /tf_static for comparison."""
    groups = {
        'ekf': [REPO / 'state_estimation/config/ekf.yaml'],
        'slam': [*sorted((REPO / 'stack_master/config/slam').glob('*.lua')),
                 *sorted((REPO / f'stack_master/config/{car}/slam').glob('*.lua'))],
        'vio': [REPO / 'state_estimation/config/vio_config.json',
                REPO / f'state_estimation/config/d435_calib_{car}.json',
                REPO / 'stack_master/config/camera/d435_config.yaml'],
        'tf': [REPO / f'stack_master/config/{car}/static_transforms.yaml'],
        'sensors': [REPO / 'stack_master/config/sensors.yaml', REPO / f'stack_master/config/{car}/imu.yaml',
                    REPO / f'stack_master/config/{car}/vesc.yaml'],
    }
    cfg = out / 'config'
    for group, files in groups.items():
        (cfg / group).mkdir(parents=True)
        for f in files:
            if f.exists():
                shutil.copy(f, cfg / group / f.name)
    (cfg / 'tf' / 'tf_static_from_bag.yaml').write_text(
        '# /tf_static as recorded in the bag, for comparison: the run used static_transforms.yaml, only the camera_* frames of this\n'
        + yaml.safe_dump(bag_static_tf(bag), sort_keys=False))
    git = {}
    for k, c in (('commit', ['git', 'rev-parse', 'HEAD']), ('branch', ['git', 'rev-parse', '--abbrev-ref', 'HEAD']),
                 ('status', ['git', 'status', '--short'])):
        try:
            git[k] = subprocess.check_output(c, cwd=REPO, text=True).strip()
        except subprocess.CalledProcessError:
            git[k] = 'n/a'
    (cfg / 'git.json').write_text(json.dumps(git, indent=2))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='bag directory recorded on the car (ros2 bag record -a)')
    ap.add_argument('--gt-bag', metavar='BAG',
                    help='second bag with the ground truth (recorded on the mocap PC); clocks are aligned in the analysis')
    ap.add_argument('--map', default='latest', help='map name in stack_master/maps (default: latest)')
    ap.add_argument('--car', default=os.environ.get('RACECAR_VERSION'), help='racecar_version of the bag (default: $RACECAR_VERSION)')
    ap.add_argument('--name', help='label appended to the result folder name')
    a = ap.parse_args()
    if not a.car:
        ap.error('--car is required when RACECAR_VERSION is not set')
    use_vio = os.environ.get('USE_VIO', 'false').lower() == 'true'

    bag = Path(a.bag).resolve()
    gt_bag = Path(a.gt_bag).resolve() if a.gt_bag else None
    gt_default = GT_TOPIC.format(car=a.car.lower())
    gt_topic = gt_default if gt_default in bag_topics(gt_bag or bag) else None
    if gt_bag and not gt_topic:
        fail(f'no ground truth topic {gt_default} in {gt_bag}')
    if gt_topic:
        print(f'ground truth: {gt_topic}' + (f' from {gt_bag}' if gt_bag else ''), flush=True)
    RESULTS.mkdir(parents=True, exist_ok=True)
    label = datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + bag.name + (f'_{a.name}' if a.name else '')
    out = RESULTS / label
    out.mkdir()
    (out / 'logs').mkdir()
    save_configs(out, a.car, bag)
    (out / 'config' / 'meta.json').write_text(json.dumps({'bag': str(bag), 'args': vars(a), 'use_vio': use_vio,
                                               'gt_topic': gt_topic,
                                               'gt_bag': str(gt_bag) if gt_bag else None, 'created': label}, indent=2))

    play_topics = PLAY + (PLAY_VIO if use_vio else [])
    record_topics = list(RECORD)
    if gt_topic and not gt_bag:
        play_topics.append(gt_topic)
        record_topics.append(gt_topic)

    procs, record, play = [], None, None
    try:
        procs = [run(['ros2', 'launch', 'state_estimation', 'state_estimation_launch.xml', f'racecar_version:={a.car}',
                      'mapping_bool:=False', f'map_name:={a.map}', 'use_sim_time:=True', f'use_vio:={str(use_vio).lower()}'],
                     stdout=open(out / 'logs' / 'launch.log', 'w'), stderr=subprocess.STDOUT),
                 run(['ros2', 'run', 'vesc_ackermann', 'vesc_to_odom_node', '--ros-args',
                      '--params-file', str(REPO / f'stack_master/config/{a.car}/vesc.yaml'), '-p', 'use_sim_time:=true'],
                     stdout=open(out / 'logs' / 'vesc_to_odom.log', 'w'), stderr=subprocess.STDOUT),
                 # sensor mounts from the config; the RealSense driver's internal tree (camera_link -> ... -> camera_imu_optical_frame)
                 # is not running, so those frames are republished from the bag
                 run(['ros2', 'launch', 'stack_master', 'static_transforms_launch.xml', f'racecar_version:={a.car}', 'sim:=False'],
                     stdout=open(out / 'logs' / 'static_tf.log', 'w'), stderr=subprocess.STDOUT)]
        if use_vio:
            procs.append(run(['ros2', 'run', 'state_estimation', 'bag_static_tf_node', '--ros-args',
                              '-p', f'bag:={bag}', '-p', 'frame_regex:=^camera_', '-p', 'use_sim_time:=true'],
                             stdout=open(out / 'logs' / 'static_tf.log', 'a'), stderr=subprocess.STDOUT))
        time.sleep(SETTLE_S)
        for p in procs:
            if p.poll() is not None:
                fail(f'{" ".join(p.args[:4])} exited with code {p.returncode}\n'
                     + (out / 'logs' / 'launch.log').read_text()[-2000:], procs)
        required_nodes = ['/ekf_filter_node', '/cartographer_node', '/imu_to_base', '/vesc_to_odom_node']
        expected_pubs = ['/imu', '/odom', '/scan', '/state_estimation/odom', '/tracked_pose']
        if use_vio:
            required_nodes += ['/basalt/vio_backend', '/basalt_to_ekf', '/bag_static_tf']
            expected_pubs += ['/basalt/odomimu', '/basalt/odom']
        check_nodes(out, procs, required_nodes)
        record = run(['ros2', 'bag', 'record', '--use-sim-time', '--compression-mode', 'message', '--compression-format', 'zstd',
                      '-o', str(out / 'replay'), *record_topics],
                     stdout=open(out / 'logs' / 'record.log', 'w'), stderr=subprocess.STDOUT)
        time.sleep(2.0)
        play = run(['ros2', 'bag', 'play', str(bag), '--clock', '500', '--topics', *play_topics])
        try:
            try:
                play.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                pass
            check_publishers(out, procs + [record, play], expected_pubs)
            play.wait(timeout=bag_duration(bag) + 60)
        except subprocess.TimeoutExpired:
            print('bag play did not finish in time, stopping it', flush=True)
        finally:
            time.sleep(1.0)      # let the last estimates through

    finally:
        stop(play)
        stop(record, timeout=120)        # flushes its cache on SIGINT, must not be killed
        for p in procs:
            stop(p)

    analyze(out, gt_topic=gt_topic, use_vio=use_vio, gt_bag=gt_bag)      # exits with a red banner when an expected topic is missing
    latest = RESULTS / 'latest'
    if latest.is_symlink() or latest.exists():
        latest.unlink()
    latest.symlink_to(label)
    print(f'\nresults: {out}\nlatest:  {latest}')
    warn_size(RESULTS)


if __name__ == '__main__':
    main()
