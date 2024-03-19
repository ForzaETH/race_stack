# Slam Tuner
This package provides functionalities to tune Localization packages.

## Launch Procedure

Start by getting your favorite rosbag and put it in the `data` folder of the `slam_tuner` folder.

Then run the `tuner.launch` launch file, for example:

```bash
roslaunch slam_tuner tuner.launch bag:=hangar/hangar_2205_speed.bag map_name:=hangar_2205_v0 racecar_version:=NUC2 algo:=pf2 include_SE:=True SE_version:=new
```

This launch file has a few parameters:
- `bag`: Relative path to the rosbag within `slam_tuner/data`
- `map_name`: Map you wish to get from `stack_master/maps`
- `racecar_version`: Choose the relevant NUC lookup tables
- `algo`: For now, choose between:
  - `slam`: Cartographer
  - `pf2`: Particle Filter 2
- `include_SE`: Whether or not to run the EKFs (early and late) or just to run localization
- `SE_version`: Choose between:
  - `new`: With late fusion EKF (and other parameters)
  - `old`: Without

The launch file should replay the Rosbag and show the mapping/localization process in Rviz. It's then up to you to tune the relevant parameters in `slam_tuner/config`.

## Tuning Parameters
### `Cartographer`
#### Mapping
When mapping, edit the `f110_2d.lua` (for Mapping) or `f110_2d_loc.lua` (for Localization) file.

Based on our trust on Odometry values:
- `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight`
- `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight`

These weight the cost of rotating/translating incoming scans (from the odometry reference). If we trust our odometry, we can weight these values a little higher, but if not (our IMU and `vesc/odom` values aren't great at the moment) the default values are `0.2`.

- `POSE_GRAPH.optimization_problem.odometry_rotation_weight`
- `POSE_GRAPH.optimization_problem.odometry_translation_weight`

These weight how much the pose graph weights odometry (over scan matching). Similarly to above, but when performing localization of the car. Default values are `0`.

Based on our trust of LIDAR scans:
- `TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability`
`TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability`

These weight the probability of a hit or miss in the occupancy grid.
- Hit probability: The trust in cells where we get range data
- Miss probability: The trust in cells where we observe free space

If we don't trust the lidar (eg. black tubes), we should reduce the miss probability (free space may mean undetected things).

#### Localization
**Work In Progress**

### `Particle Filter 2`
There are a few parameters to tune for PF2.

- `max_particles`: The number of particles in PF. Do not increase to 4000, as something really wacky goes on with CPU usage past that point. Tested 2000-3000 and it was safe.
  - Increasing the number of particles should improve the smoothness and accuracy of the pose estimate as there are more proposals.
- `max_range` : The maximum range to trust lidar scans. If encountering black tubes, then perhaps it is wise to tune this value lower. Else, trust the lidar datasheet.

Sensor Model Parameters.

- `z_hit`: Probability we hit intended target.
- `z_short`: Unexpected short reading. This could be turned up for H2H mode.
- `z_max`: out-of-range reading beyond `max_range`. This should be turned up for black tubes.
- `z_rand`: a reading anywhere in the valid range.

Note that these values should sum to 1.

Model Variances.

- `sigma_hit`: standard deviation (m) of hitting intended target
- `lambda_short`: parameter of short-reading exponential distribution. Increase to make short readings more likely.

Motion Model Parameters. Here are the TUM Motion Model equations:

Where $d_{r1}$, $d_{r2}$ are the first and second rotation distances in the diff-drive model; and $d_t$ is the translation distance. Note that $a_2$ needs to be changed drastically if changing the Motion Model to "amcl".

$\sigma_{r1} = a_1 * d_{r1} + a_2/max(d_t,\lambda_t)$

$\sigma_{r2} = a_1 * d_{r2} + a_2/max(d_t,\lambda_t)$

$\sigma_{t} = a_3 * d_{t} + a_4 * (d_{r1} + d_{r2})$

- `alpha_1`: How rotation affects rotation variance.
  - Recommended to keep between (0.0, 1.0)
- `alpha_2`: How translation affects rotation variance.
  - Recommended to keep between (0.0, 0.05)
  - Coupled with `lam_thresh`: change these values in unison.
- `alpha_3`: How translation affects translation variance 
  - Recommended to keep between (0.0, 5.0)
  - If facing localization problems at the end of the straight, think about increasing this.
  - It will make particles spread out further longitudinally.
- `alpha_4`: How rotation affects translation variance
  - Recommended to keep between (0.0, 1.0)
- `lam_thresh`: Minimum translation between frames for the TUM model to become effective.
  - If this is set lower, then rotational variance will become higher, see the equation for $\sigma_{r1}$ above.
  - Recommended to keep between(0.01, 0.2)
  - Generally calculating $a_2/\lambda_t=0.2$ is a safe place to start.

---
### Trajectory comparison
Navigate to `slam_tuner/profiling/traj_cmp` and take a look at the `Comparison Index.txt` to compare the trajectories. Provisionally, the bags inside compare MIT PF, PF2, and Cartographer with the reference pose (what was recorded on the bag). An example is below. Replace the Bash variables with the relevant equivalents from `Comparison Index.txt`

```bash
BAGNAME="pf_pf2_cartographer_v3_filt"
REF_POSE="/car_state/pose"
PF_POSE="/tracked_pose"
PF2_POSE="/tracked_pose/pf2"
CARTO_POSE="/tracked_pose/cartographer"
evo_traj bag $BAGNAME $PF_POSE $PF2_POSE $CARTO_POSE --ref $REF_POSE -p --plot_mode xy --verbose
```

## Profiling C++ ROS Nodes:
Follow the [profiling guide](http://wiki.ros.org/roslaunch/Tutorials/Profiling%20roslaunch%20nodes) on the ROS wiki:
- The C++ CMakeLists.txt has to be exposed to you:
  - For Cartographer especially, we already have it;
  - For `slam_toolbox` and `amcl`, we would need to clone the source code from Github and re-build from source with the necessary debug flags.
  - Add the `-pg` flag -- see details in the guide.

### Using `sysprof`
Sysprof is a *system-wide* profiler for Linux, and it's a general way to profile the different algorithms we use as part of our software stack.

First, install `sysprof`: `sudo apt install sysprof`.

Next, we can start to profile the system:
1. Source the latest version of the workspace.
2. Get a launchfile ready, for instance:
    ```bash
    launch_cmd="roslaunch slam_tuner tuner.launch bag:=hangar/hangar_240323_racerehearsal.bag loc_only:=False map_name:=hangar/hangar_240323_racerehearsal mapping_algo:=toolbox"
    ```
3. Start up the profiler. `sysprof` will ask you for sudo rights (it is a profiler after all).
    ```bash
    bash profiling/start_sysprof.bash "$launch_cmd" "$OUTFILE"
    ```
    The bash script starts `sysprof` and starts the program to be profiled.
4. Kill the profiling bash script when you are happy (for example, after 30 seconds).
5. Open the `sysprof` GUI on your computer. Note that sudo is needed to do it directly.
    ```bash
    sudo sysprof $OUTFILE
    ```
    Alternatively, just run `sysprof` and open up `OUTFILE` with `Ctrl+O`.
6. Enjoy a cool call graph and stack traced code.

**! Note:** Sysprof profiles *everything* running on the system. So if you want the most accurate tests, close everything else running.

**TODO** - isolate only the commands launched in `-c ...` for the Bash script.

Alternatively, to get an isolated process outfile:
1. Start the launch file in question
2. Find the PID of the process you want to profile, eg. running `ps aux | grep "\.ros"` and reading off the PID
3. Run `sysprof-cli -p $PID --memprof $OUTFILE --force`

### Using `gprof`
Add an <env> tag just before each <node> tag. The GMON_OUT_PREFIX environment variable specifies the output name for gprof, so it should be identical.
```xml
<env name="GMON_OUT_PREFIX" value="toolbox" />
<node pkg="slam_toolbox" type="async_slam_toolbox_node" name="slam_toolbox" output="screen">
    <rosparam command="load" file="$(find slam_tuner)/config/slam_toolbox_online_async.yaml" />
</node>
```

In this case, the ROS node will output the profiling output data to you ~/.ros/ directory with file named toolbox.[pid].

To analyse data, run `gprof ~/catkin_ws/devel/lib/slam_toolbox/async_slam_toolbox_node gmon.out`. The first argument is the executable and the second argument is the output dumped by `gmon`.

### Using `valgrind`
**Valgrind makes the program quite slow, and I've not been able to get a replay working on rosbags using this method. Try it, but don't expect it to work.**

Add the following code before the ROS node you wish to profile:
```xml
<node pkg="slam_toolbox" type="async_slam_toolbox_node" name="slam_toolbox" output="screen"

launch-prefix="valgrind --tool=callgrind --callgrind-out-file='callgrind.slam_toolbox.%p'"
>
```

A file will then be output in `~/.ros/` with the filename specified and the PID of the node. It can then be analysed with `kcachegrind`.

---
## Evaluating Localization algorithms

### Data logs from runs
When `tuner.launch` is run, we also run a script called `reconstruction_error` (found in src). This is a slightly bad name, but this script basically records the localization performance for later use.

After running the rosbag in `tuner.launch` (details at the top of the README), you will want to save the accumulated data for comparison. To do so, run the following in a **new terminal**:

```bash
rostopic pub /create_plot std_msgs/Empty --once
```

This sends a message to the `reconstruction_error` node, telling it to dump the following files to `slam_tuner/profiling`:
- Three CSV files with format `(x, y, theta, timestep)`:
  - Cartographer `tracked_pose` topic, if available
  - The `Map->Base_link` TF transform
  - The `Map->Odom` TF transform
  - The `Odom->Base_link` TF transform
- A plot of trajectory histories of `Map->Base_link`, `Map->Odom`, and `Odom->Base_link` TF transforms
- One CSV file with format `(timestep, odom_x, odom_y, odom_theta, tf_x, tf_y, tf_theta)`. The TF values here are the `Map->Base_link` TF transform.
- A plot of localization error between the `Map->Base_link` TF transfom and the Odometry source. This would be helpful when we get a ground-truth odometry source, (eg. VICON system).

These named `err_metrics_<LOC_ALGO>_DDMMYY_HHMMSS_<FILENAME>`. The filename should be obvious based on the above labels.


### Evaluating trajectories

To evaluate trajectories, try running `profiling/metrics.py`. Note the relative import in the first few lines; you will need to adjust the include path manually.

The script takes in a few variables:
- Paths to CSV files to be analysed
- Time interval to analyse (`START_TIME`, `END_TIME`)
- Etcetra.

The "smoothness" metric can take a few options, and is still something TBD:
- `"SUM_SECOND_DERIVATIVE"`: Find the numerical second derivative of the `x` and `y` axes with respect to the timestamp `t` and report the sum of squared instantaneous second derivatives.
- `"SPLINE_DERIVATIVES"`: Fit a 5th order spline through sequential subsections of the data, and sum their squared second derivatives.
  - Takes in two more parameters:
    - `time_interval`: Duration of the entire spline (i.e. `END_TIME-START_TIME` as defined earlier)
    - `dt`: Time step of sequential sections of data.
  - This is because splines of order 5 sometimes are unable to fit the entire dataset well.
- `"SPLINE_DEVIATION"`:
  - Similar to above, tries to fit a nth order spline through the datapoints, but the error metric is the squared deviation from the fitted spline.
- Anything else (Default argument):
  - Angle smoothness metric (how the angle between successive data points changes, weighted by the distance of the edges).

These metrics will then be plotted at the end of the script.

---
[Go back to the utils list](../../README.md)