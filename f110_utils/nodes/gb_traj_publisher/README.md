# Global Trajectory Republisher
This package contains the global trajectory republisher which currently plays the central role of providing the trajectory to the racing system.

Currently the trajectory is the re-published in the following way:
  1. During mapping, a rosbag is recorded, so that when the trajectory is computed it will be recorded with the bag
  2. When the `base_system` is launched, the rosbag is played and the `global_trajectory_publisher` is launched. This node reads the trajectory and then periodically republishes the trajectory even after the bag stops playing.

How to use the ellipse map for experiments:
  1. SSH into the car and launch teleop with `roslaunch racecar pbl_teleop.launch racecar_version:=NUC2`. 
  2. Launch `roslaunch gb_traj_publisher gb_traj.launch exp:=ellipse` to get the ellipse trajectory. Also open rviz via the pitstarter script.
  3. Add the topic `/gb_traj_markers` to see the ellipse.
  4. Place some objects arround the ellipse such that SLAM locilisation works.
  5. Create a folder at `~/catkin_ws/src/race_stack/stack_master/maps`.
  6. Drive one lap to map.
  7. Call `./finish_map.sh ~/catkin_ws/src/race_stack/stack_master/maps/test/test.pbstream` with the right path to the above created folder.
  8. Now the pbstream is saved. We now run `roslaunch racecar pbl_teleop.launch racecar_version:=NUC2 loc_only:=True map_name:=test`. You might have to copy existing ot_sector and speed_scaling yaml files to make it work.
  9. Relaunch the `roslaunch gb_traj_publisher gb_traj.launch exp:=ellipse` to publish the trajectory. Take this trajectory for the controller.

  ---
[Go back to the utils list](../../README.md)
