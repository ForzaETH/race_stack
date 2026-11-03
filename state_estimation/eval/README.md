# State estimation evaluation

Headless replay of a bag through the state estimation stack (EKF, cartographer, optionally
Basalt) followed by an offline analysis with [evo](https://github.com/MichaelGrupp/evo).
Everything of one run lands in a timestamped folder, `results/latest` points to the newest.

```shell
python3 state_estimation/eval/run_eval.py bags/<bag> [--map <map_name>] [--car NUCx] [--name ekf_v2]
```
Needs `rosbags` and `evo` (in `.install_utils/python_req.txt`, so part of the image; in an older
container: `pip3 install rosbags evo`).

Defaults: `--map latest`, `--car $RACECAR_VERSION`. `USE_VIO` decides whether Basalt runs on the bagged
images or the bagged `/basalt/odom` is used. `--static-tf bag|config` (default `bag`: `/tf_static` as
recorded; `config`: mounts from `static_transforms.yaml`). `--vesc-odom bag|config` (default `bag`: `/odom` as
recorded; `config`: recomputed by `vesc_to_odom_node` from `/sensors/core` with `vesc.yaml`, to test
`speed_to_erpm_gain` or `vesc_to_odom` changes). `--scan-offset S` shifts the bagged `/scan` stamps by S seconds
(emulates `default_user_latency` of `urg_node`, which cannot run in replay). Re-run only the analysis of an existing run
(e.g. after changing `analyze.py`): `python3 state_estimation/eval/analyze.py results/<run>`.

## What it does
1. `ros2 launch state_estimation state_estimation_launch.xml use_sim_time:=True ...` (the same launch
   as on the car; with `--static-tf config` additionally `static_transforms_launch.xml` and the
   RealSense TF tree from the bag)
2. `ros2 bag record --use-sim-time` of the outputs and inputs into `replay/`
3. `ros2 bag play --topics <raw sensors>` and waits until the bag is done
4. copies the configs into `config/ekf` (`ekf.yaml`), `config/slam` (`.lua`), `config/vio` (Basalt config,
   calibration, camera), `config/tf` (`static_transforms.yaml`, or `tf_static_from_bag.yaml` dumped from the bag in the default `--static-tf bag` mode), `config/sensors` (`sensors.yaml`, `vesc.yaml`)
   plus the git state (`config/git.json`: commit, branch, dirty files)
5. `analyze.py`: plots, data exports + `metrics.json`, `summary.md`

Only raw sensor topics are played (`/scan`, `/sensors/imu/raw`, camera, and `/odom` or the raw VESC
state), so every output is recomputed with the configs currently in the workspace (symlink-install:
edit, run, compare).

## Result folder
```
results/<YYYYmmdd_HHMMSS>_<bag>[_<name>]/
  summary.md                           the numbers, also printed to the console
  config/{ekf,slam,vio,tf,sensors}/    configs the run was made with, config/git.json, config/meta.json (args)
  data/                                carto/ekf/basalt/gt as .tum (evo CLI), all signals as .csv, metrics.json
  logs/                                launch.log, record.log, vesc_to_odom.log, static_tf.log, scan_offset.log
  plots/
    trajectories.png        cartographer in map, EKF + Basalt in odom (+ GT aligned)
    global_consistency.png  map->odom over time, its correction rate, /tracked_pose discontinuities
    ekf_rpe.png             EKF relative pose error over 1 s vs cartographer (or GT): local accuracy
    velocities.png          vx / vy / yaw rate: VESC, Basalt, gyro, EKF, differentiated cartographer (or GT)
    yaw.png                 yaw of all sources + residual of the TF chain vs /tracked_pose
    imu.png                 /imu gyro and accel (base_link)
    timing.png              header stamp intervals and record-time minus header-stamp latency per topic
    carto_ape_gt.png        (with GT) cartographer absolute pose error
  replay/                              recorded bag of the run
```

## Metrics
* global (cartographer): rate of `map->odom` corrections (how much cartographer fights the EKF),
  total yaw drift of the odom frame, jumps in `/tracked_pose` beyond what the velocity explains,
  APE vs ground truth after SE(3) alignment.
* local (EKF): evo RPE over 1 s against cartographer (frame independent, so the drifting odom frame
  does not matter) or ground truth; velocity errors of the EKF against the reference; residual between
  the TF chain `map->odom->base_link` and `/tracked_pose`.
* inputs: each raw velocity signal against the reference velocity, time offset of each signal against
  the reference by cross-correlation (catches wrong stamps: + means the signal's stamps are early),
  IMU standstill means (bias), rates, max gaps, non-monotonic stamps, latency.

## Ground truth
`GT_TOPIC` in `run_eval.py` (placeholder `/vicon/car`): when the bag contains it, it is played, recorded
and used as the reference instead of cartographer. Supported message types: `PoseStamped`, `Odometry`, `TransformStamped`,
`PoseWithCovarianceStamped` (see `pose_of()` in `analyze.py`). The GT frame does not have to be
the map frame, cartographer is aligned to it (Umeyama, no scale) before the APE. Time offsets between
GT and the car are not estimated, stamps must be on the same clock.

## Disk usage
A run stores roughly 3-5 MB per minute of bag (zstd-compressed `replay/`, `/scan` and `/sensors/imu/raw` are not recorded),
plus a few MB of plots and CSVs. The driver prints the size of `results/` after each run and warns
above 1 GB; delete old runs or just their `replay/` folders.

## Caveats
* Reference velocities without GT are differentiated `/tracked_pose` (Savitzky-Golay, 0.15 s window),
  good enough for lag and scale checks, noisy at the mm level.
* Latency in `timing.png` is "record time minus header stamp" with the recorder on sim time.
  Nodes stamping at receive time (VESC IMU) show ~0.
* `analyze.py` needs no ROS, it reads the bag with `rosbags`. It runs on any bag with the topics,
  e.g. a bag recorded on the car: `python3 analyze.py <out_dir> --bag bags/<bag>`.
