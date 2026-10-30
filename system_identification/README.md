# System Identification

Lateral (tire) identification lives in `lateral/`:

- `on_track_sysid/`: git submodule of [On-Track-SysID](https://github.com/ForzaETH/On-Track-SysID), the ROS-free algorithms (on-track and steady-state identification, steering LUT generation).
- `lateral_sysid_ros/`: ROS 2 nodes and tools around it. They read and write everything through `forza_config_ros`.
- `steering_lookup/`: the LUT reader used by the MAP controller.

Results go to `stack_master/config/<racecar_version>/`:

- `pacejka/<floor>/default.yaml`: the identified tires.
- `LUT/<floor>/default.csv`: the steering look-up table.
- `pacejka/<floor>/data/`: fit plots and, optionally, raw data.

`default.*` are symlinks into `archive/`, so earlier results stay available. `car_model.yaml` (mass, inertia, geometry, actuator mappings) is maintained by hand.

## Setup

```shell
git submodule update --init system_identification/lateral/on_track_sysid
rosdep install --from-paths ~/ws/src --ignore-src -y   # torch (python3-torch), only needed for on-track identification
```

## On-track identification (about a minute of driving)

1. Start `base_system` with the right `racecar_version` and `floor`. Start any controller that drives the car.
2. Run:
   ```shell
   ros2 launch lateral_sysid_ros on_track_sysid_launch.xml
   ```
3. Drive normally. The node collects `data_duration` (default 50 s) of samples above 1 m/s from `/state_estimation/odom` and `/drive`.
4. Wait for training to finish. The node then writes the tires and the LUT, and exits.
5. Check the fit plots in `pacejka/<floor>/data/`, then commit the new config.

Parameters are in `lateral_sysid_ros/config/on_track.yaml`. To identify from a recorded bag instead, start `base_system` and run the launch file with `use_sim_time:=true`. Then replay the bag with `ros2 bag play <bag> --clock`.

## Steady-state identification (needs space, gives better tire data)

1. Start `base_system`, then record a bag:
   ```shell
   ros2 bag record /state_estimation/odom /drive /imu
   ```
   Add `/vesc/commands/servo/position` if you want to use the servo commands.
2. Drive at constant speed with a slow steering ramp:
   ```shell
   ros2 launch lateral_sysid_ros steering_ramp_launch.xml
   ```
   Settings are in `config/steering_ramp.yaml`. Stop the bag when the node warns.
3. Identify:
   ```shell
   ros2 run lateral_sysid_ros steady_state_from_bag <bag> [<bag> ...] --car <NUCx> --floor <floor>
   ```
   Pass `--servo-topic /vesc/commands/servo/position` to use servo commands instead of `/drive`. Pass `--imu-topic ''` when there is no IMU in the car frame. Pass `--vy-at-cog` for simulation data.
4. Check `pacejka/<floor>/data/steady_state_fit.png`, then commit the new config.

## Maximum steering angle

1. Center the servo mechanically. Then find the servo positions where each side reaches full lock without straining the servo, and write them as `servo_min` / `servo_max` in `vesc.yaml`. Make sure full stick reaches them: widen `steering_min` / `steering_max` and the joystick `scale` if needed.
2. Start `base_system` and record a bag:
   ```shell
   ros2 bag record /car_state/pose /imu
   ```
   Meanwhile, drive slowly (about 0.5 m/s) at full lock: a few continuous circles to one side, then a few to the other.
3. Run:
   ```shell
   ros2 run lateral_sysid_ros max_steering_from_bag <bag> --car <NUCx> [--debug]
   ```
   It warns if the two radii differ (re-center the servo) or if you drove too fast. `--debug` plots the fits.
4. Copy the printed values into `car_model.yaml`, `vesc.yaml`, `*_mpc_params.yaml` and `joy_teleop.yaml`, then commit. `stack_master/test/test_configs.py` checks that the MPC steering limits match `vesc.yaml`.

## Speed to erpm

`speed_to_erpm_gain` in `vesc.yaml` turns the commanded speed into motor erpm and the measured erpm back into the odometry speed. The offset stays fixed; only the gain is identified.

1. Start `base_system` and record a bag:
   ```shell
   ros2 bag record /car_state/pose /sensors/core
   ```
   Meanwhile, drive slowly (0.5 to 1.5 m/s) and mostly straight, at a steady speed, for at least half a minute in total. Faster driving or hard acceleration makes the wheels slip, so those samples are left out.
2. Run:
   ```shell
   ros2 run lateral_sysid_ros speed_to_erpm_from_bag <bag> --car <NUCx> [--debug]
   ```
   It compares the localized speed with the motor erpm, both averaged over 1 s, on steady, nearly straight samples. `--debug` plots the selection and the fit.
3. Copy the printed `speed_to_erpm_gain` into `vesc.yaml`, then commit. The `actuation_manager` `speed_min` / `speed_max` must stay within the printed range; `stack_master/test/test_configs.py` checks it.

## In simulation

Both methods work with `racecar_version:=SIM`. The sim odometry reports `v_y` at the center of gravity: `on_track_node` handles that automatically for SIM, and `steady_state_from_bag` needs `--vy-at-cog`. The sim publishes no IMU, so also pass `--imu-topic ''`. The sim simulates the Pacejka tires of `config/<racecar_version>/pacejka/<floor>`, so identification in simulation should recover them.

## Plot the tires

```shell
ros2 run lateral_sysid_ros pacejka_visualizer --racecar_version <NUCx> --floor <floor>
```

## Regenerate the LUT only

After editing a `pacejka/<floor>/default.yaml` by hand:

```shell
ros2 run lateral_sysid_ros generate_lut --car <NUCx> --floor <floor>
```
