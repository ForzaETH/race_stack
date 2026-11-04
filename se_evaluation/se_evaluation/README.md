# README
# SE Evaluation: Comparing Stack V1 and V2

Takes a recorded rosbag as input to a stack and compares the outputs of two stack versions when fed with identical 
rosbag input.

## Requirements
- ROS 2 Humble, Python 3.10
- `pip install rosbags pandas numpy matplotlib`

## Workflow
1. Play the input bag with `ros2 bag play <bag> --clock` and run the stack with `use_sim_time:=true`.
2. Record the stack output with `ros2 bag record --use-sim-time <output topics>` (Run1 = V1, Run2 = V2).
3. Convert the output bags to CSV/NPZ: `python3 rosbag_recordings_input.py`
4. Compare the runs (RMSE, MAE, plots): `python3 compare.py`

## Folder Structure
- `bags/Run1`, `bags/Run2`: recorded output bags (one folder per run, containing `metadata.yaml` and the `.db3` file)
- `bags/results/Run1/converted/`: exported CSV/NPZ files, one per topic
- 

## Data Format
Each CSV file corresponds to one topic (`/` in the topic name becomes `_`).
The first column `time_s` is the bag timestamp in seconds. Nested message
fields are flattened with dots (e.g. `twist.twist.linear.x`).

| File | Column | Meaning | Unit |
|---|---|---|---|
| car_state_odom.csv | twist.twist.linear.x | Longitudinal velocity | m/s |
| car_state_odom.csv | twist.twist.angular.z | Yaw rate | rad/s |
| steering.csv | data | Steering angle (verify) | rad |
| commands_motor_speed.csv | data | Motor speed command (verify, may be ERPM) | m/s |
| min_car_distance_to_boundary.csv | data | Minimum distance to track boundary | m |

LaserScan topics are stored as `.npz` (arrays: `time`, `ranges`).
Image and PointCloud2 topics are skipped.

## Notes
- The custom type `rs_msgs/msg/OvertakingSectors` is unknown and is skipped.
- Custom message types are registered from `utilities/libraries/f110_msgs/msg`
  under both the names `rs_msgs` and `f110_msgs` (the package was renamed
  after the bags were recorded).
- Output timestamps are similar but not identical between runs, so the
  comparison script interpolates both runs onto a common time grid.
- Do not commit bags or exported data to Git (add `*.db3`, `*.bag`, `converted_output/` to `.gitignore`).