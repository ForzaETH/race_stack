# Stack Master
Here is the `stack_master`, it is intended to be the main interface between the user and the PBL ForzaETH F110 system.

### Building inside the Docker container on the real car
Skip the `f110_gym` and `f1tenth_gym_ros` packages when building inside the Docker container on the real car, otherwise the build will fail:
``` bash
colcon build --packages-skip f110_gym f1tenth_gym_ros --symlink-install
```

### Mapping (on the real car)
Run the mapping launch file, specifying the map name and the NUCX version:
```shell
ros2 launch stack_master mapping_launch.xml racecar_version:=<NUCX used> map_name:=<map name of choice>
```

  - `<map name of choice>` can be any name with no white space. Conventionally we use the location name (eg, 'hangar', 'ETZ', 'icra') followed by the day of the month followed by an incremental version number. For instance, `hangar_12_v0`.
  - `<NUCX>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car).

After completing a lap, a GUI will popup and pressing the requested button will start the global raceline generation. 
Then two GUIs will be shown, and within them a slider can be used to select the sectors. 
Be careful as once a sector is chosen it cannot be further subdivided. 

A ROS resourcing will be needed from here on. 

### Changing Initial Pose After Mapping for Cartographer

By default, Cartographer initializes localization at the **map origin** — the exact spot where you started the mapping run. If you place the car at a different location on the track for time-trials or h2h, Cartographer may fail to localize itself sometimes, especially if there are not enough landmarks nearby.

The **Initial Pose Bridge** node solves this by letting you click on the map in RViz to tell Cartographer where the car actually is.

#### How It Works

When you click "2D Pose Estimate" in RViz, the bridge node:
1. Finishes the current Cartographer localization trajectory
2. Starts a new trajectory at the clicked position
3. Cartographer immediately begins scan-matching from that location

### Steps

1. **Launch the base system** as normal:
   ```bash
   ros2 launch stack_master base_system_launch.xml map_name:=MY_TRACK racecar_version:=NUC2
   ```
   The Initial Pose Bridge node starts automatically alongside Cartographer.

2. **Open RViz** on your laptop (or on the car):
   ```bash
   rviz2 -d $(ros2 pkg prefix stack_master --share)/viz/head_to_head.rviz
   ```
   Or just run `rviz2`, set the Fixed Frame to `map`, and add a Map display on the `/map` topic.

3. **Look at the real car** and identify approximately where it is on the physical track.

4. **Click "2D Pose Estimate"** in the RViz toolbar (top bar), then click and drag on the map at the car's location. The drag direction sets the heading.

5. **Verify in the terminal** — you should see output like:
   ```
   [initialpose_bridge] Received initial pose: x=5.20, y=3.10, yaw=0.79 rad
   [initialpose_bridge] Finishing trajectory 1...
   [initialpose_bridge] Starting new trajectory at clicked pose...
   [initialpose_bridge] Started new trajectory 2
   ```

6. **Verify in RViz** — the robot's position should jump to where you clicked and the lidar scan should align with the map walls.

7. **Start driving** — Cartographer will continue localizing from the correct position.

> **Note:** You can click "2D Pose Estimate" multiple times if needed. Each click restarts the localization trajectory from the new position.

### Base System
```shell
ros2 launch stack_master base_system_launch.xml map_name:=<name of mapped track> sim:=<true/fasle> racecar_version:=<NUCX used>
```
  - `<name of mapped track>` is the name of the track you want to run on. It must belong to the list of maps available in the `stack_master/maps` folder.
  - `<true/false>` is a boolean value that indicates if you want to run the simulation or the real car. 
  - `<NUCX>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car).

### Time trials 
```shell
ros2 launch stack_master time_trials_launch.xml racecar_version:=<NUCx used> LU_table:=<Look-Up Table name> ctrl_algo:=<control algorithm> 
```
  - `<NUCx>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car).
  - `<Look-Up Table name>` is the name of the Look-Up Table you want to use. It must belong to the list of Look-Up Tables available in the `systm_identification/steering_lookup/cfg` folder.
  - `<control algorithm>` is the control algorithm you want to use. Current possibilities are MAP / PP.

### Head to Head
```shell
ros2 launch stack_master head_to_head_launch.xml racecar_version:=<NUCx used> LU_table:=<Look-Up Table name> ctrl_algo:=<control algorithm> overtake_mode:=spliner
```
- `<NUCx>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car).
- `<Look-Up Table name>` is the name of the Look-Up Table you want to use. It must belong to the list of Look-Up Tables available in the `systm_identification/steering_lookup/cfg` folder.
- `<control algorithm>` is the control algorithm you want to use. Current possibilities are MAP / PP.
- `<overtake_mode>` is the mode you want to use for overtaking. `spliner` is the only current possibility.

To launch an opponent in simulation for testing, refer to  [`opponent_publisher` README](./../utilities/nodes/opponent_publisher/README.md).
