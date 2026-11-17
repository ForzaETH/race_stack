# State estimation evaluation

Headless replay of a bag through the state estimation stack (EKF, cartographer, optionally
Basalt) followed by an offline analysis with [evo](https://github.com/MichaelGrupp/evo).
Everything of one run lands in a timestamped folder, `results/latest` points to the newest.

```shell
USE_VIO=true python3 state_estimation/eval/run_eval.py bags/<bag> --gt-bag bags/<gt> [--map <map>] [--car NUCx] [--name ekf_v2]
```

Defaults: `--map latest`, `--car $RACECAR_VERSION`, `--gt-bag` optional (see Ground truth). `USE_VIO=true` runs Basalt on the
bagged images, `USE_VIO=false` runs the EKF on VESC + IMU only (the bagged `/basalt/odom` is never used). `/odom` is always
recomputed by `vesc_to_odom_node` from `/sensors/core` with the workspace `vesc.yaml`, and `/tf_static` is published from
`static_transforms.yaml` (only the RealSense-internal `camera_*` frames come from the bag), so sensor mounts can be tuned offline.
Re-run only the analysis of an existing run (e.g. after changing `analyze.py`):
`python3 state_estimation/eval/analyze.py results/<run>` (GT topic/bag and `USE_VIO` are taken from the run's `config/meta.json`).

## What it does
1. `ros2 launch state_estimation state_estimation_launch.xml use_sim_time:=True ...` (the same launch
   as on the car) + `vesc_to_odom_node` + `static_transforms_launch.xml` (+ `bag_static_tf_node` for the camera tree)
2. `ros2 bag record --use-sim-time` of the outputs and inputs into `replay/`
3. `ros2 bag play --topics <raw sensors>` and waits until the bag is done
4. copies the configs into `config/ekf` (`ekf.yaml`), `config/slam` (`.lua`), `config/vio` (Basalt config,
   calibration, camera), `config/tf` (`static_transforms.yaml` as used, `tf_static_from_bag.yaml` for comparison), `config/sensors` (`sensors.yaml`, `imu.yaml`, `vesc.yaml`)
   plus the git state (`config/git.json`: commit, branch, dirty files)
5. `analyze.py`: plots, data exports + `metrics.json`, `summary.md`

Only raw sensor topics are played (`/scan`, `/sensors/imu/raw`, `/sensors/core`, camera), so every output is recomputed with the configs currently in the workspace (symlink-install:
edit, run, compare).

## Result folder
```
results/<YYYYmmdd_HHMMSS>_<bag>[_<name>]/
  summary.md                           the numbers, also printed to the console
  config/{ekf,slam,vio,tf,sensors}/    configs the run was made with, config/git.json, config/meta.json (args)
  data/                                carto/ekf/basalt/gt as .tum (evo CLI), all signals as .csv, metrics.json
  logs/                                launch.log, record.log, vesc_to_odom.log, static_tf.log
  plots/
    trajectories.pdf        cartographer in map, EKF + Basalt in odom (+ GT aligned)
    global_consistency.pdf  map->odom x, y and yaw over time
    ekf_rpe.pdf             EKF relative pose error over 1 s vs cartographer (or GT): local accuracy
    velocities.pdf          vx / vy / yaw rate: VESC, Basalt, gyro, EKF, differentiated cartographer (or GT)
    yaw.pdf                 yaw of all sources
    timing.pdf              header stamp intervals and record-time minus header-stamp latency per topic
    carto_ape_gt.pdf        (with GT) cartographer absolute pose error
    laps/trajectory_<i>.pdf one plot per lap (return to the window's first position): reference + each odometry source
                            aligned onto it over that lap only, with the per-lap APE

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
`GT_TOPIC` in `run_eval.py` (`/<car>/pose`, e.g. `/nuc6/pose`):
when the bag contains it, it is played, recorded and used as the reference instead of cartographer. With `--gt-bag BAG`
the topic is read from that second bag instead (recorded on the mocap PC while the car records its own sensors). Supported message types: `PoseStamped`, `Odometry`, `TransformStamped`,
`PoseWithCovarianceStamped` (see `pose_of()` in `analyze.py`). The GT frame does not have to be
the map frame, cartographer is aligned to it (Umeyama, no scale) before the APE. A constant clock offset
between the mocap PC and the car (up to ±5 s) is estimated by cross-correlating the GT yaw rate with the gyro and
removed (`gt_clock_offset_s` in metrics.json). A `<gt>/twist` topic next to the pose (`TwistStamped`, world frame) is
used as the linear velocity reference when present (rotated into the body frame with the GT yaw, shifted by half a sample
because the bridge stamps a backward difference at its end), otherwise the pose is differentiated. The yaw rate always comes
from the differentiated pose: the bridge's `angular.z` is clipped at ±π and reads ~20 % low above 2.8 rad/s. It is still the
noisiest reference signal (~0.15–0.3 rad/s rmse against the gyro, growing with yaw rate; Vicon stamps are receive times with
~5 ms jitter), so the `vyaw` rows are limited by the GT, not by the EKF. Tracking losses (stamp gaps > 0.1 s, `gt_tracking_gaps`
in metrics.json) are masked in the reference velocities, widened by the smoothing window. With GT, the "inputs vs reference" block
and the time offsets are against the GT as well. The GT object origin should sit at base_link (rear axle centre):
a position offset is removed by the alignment, but it adds yaw_rate x offset to the GT lateral velocity.

## Some info
* All numbers are taken on a window starting `WARMUP_S` (3 s) after the car first moves, and after the last map->odom
  jump (> 0.5 m or > 10 deg) within the first `INIT_S` (10 s) of cartographer: a relocalisation at start-up is
  initialisation, not a loss (`window` in metrics.json).
* Reference velocities are Savitzky-Golay smoothed over `VEL_WINDOW_S` (0.25 s), the reference heading over
  `YAW_WINDOW_S` (0.3 s) before rotating into the body frame (heading jitter leaks speed into vy). Longer windows
  cut the jitter further but smear the 0.2-0.3 s turn-in transients. Without GT the reference is the differentiated output of cartographer.
* Latency in `timing.pdf` is "record time minus header stamp" with the recorder on sim time. Some nodes, e.g. vesc imu node, stampt at receive time so it will always show ~0.