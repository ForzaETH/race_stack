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
pip install torch   # only needed for on-track identification
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

## In simulation

Both methods work with `racecar_version:=SIM`. The sim odometry reports `v_y` at the center of gravity: `on_track_node` handles that automatically for SIM, and `steady_state_from_bag` needs `--vy-at-cog`. The sim publishes no IMU, so also pass `--imu-topic ''`. The sim uses linear tires, so the identified Pacejka curve matches them only inside the slip range that was driven.

## Regenerate the LUT only

After editing a `pacejka/<floor>/default.yaml` by hand:

```shell
ros2 run lateral_sysid_ros generate_lut --car <NUCx> --floor <floor>
```
