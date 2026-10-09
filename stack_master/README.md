# Stack Master
Here is the `stack_master`, it is intended to be the main interface between the user and the PBL ForzaETH F110 system.

## Zenoh
We use [zenoh](https://github.com/ros2/rmw_zenoh) as ros2 middleware. The zenoh router is started as a vscode task when you open the container. You can also start it manually with: `ros2 run rmw_zenoh_cpp rmw_zenohd` or the `zenoh` alias.
If you have problems you may need to first stop the ros2 daemon: `ros2 daemon stop`

### Connect to host
If you now want to connect to this host (e.g. to stream topics to RViz on another computer) you can run the following command on the other computer: `export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/THE_CAR_IP:7447"]'`. You will need to run this command in each terminal with which you want to connect. You can also add the ips of the cars to `remote_car.sh` and run `scripz && source remote_car.sh YOUR_CAR`.



### Parameters
There are two important parameters when running the base system or mapping:
 - racecar_version: This sets car related parameters such as TFs, lookup tables or Pacejka Parameters. By default this is written to the `.env` file in the installation progress.
 - map_name: This is the name of the map that you want to use / create. Every map is saved in `stack_master/maps/MAP_NAME/`. However, by default a map called `latest` is used.

### Mapping (on the real car)
Run the mapping launch file:
```shell
ros2 launch stack_master mapping_launch.xml map_name:=<map name of choice>
```
  - `<map name of choice>` can be any name with no white space. Conventionally we use the location name (eg, 'hangar', 'ETZ', 'icra') followed by the day of the month followed by an incremental version number. For instance, `hangar_12_v0`. You can also omit this argument. Then a map called `latest` is created. If there already is such a map it is copied to `backup`.
  - `<NUCX>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car). If you have exported a `.env` file during the build process this is read from the env variables and does not have to be set.

If you have a display connected (we recommend [this tool](https://github.com/ETH-PBL/remote-novnc)) an image will pop up showing you the current map. In the image you can press a button to save it. If you have no display you can do that via the terminal by pressing `y`, once you are satisfied by the map.



### Global Planner (on the real car)
After mapping we can run the global planner. Before you can check the saved png and edit it with your favorite image editing software. This allows you to remove mapping artefacts or add new virtual chicanes to the map.
Then you can run the global planner:
```shell
ros2 launch stack_master planner_launch.xml map_name:=<map name of choice>
```
 - `<map_name_of_choice` must be the name of any prerecorded map. If it is ommited the map called `latest` will be used.

A GUI will popup showing you the map and the extracted centerline. If you are satisfied you can close it to start the global raceline generation. If you have no monitor it starts automatically. Then two GUIs will be shown, and within them a slider can be used to select the velocity scaling and overtaking sectors. 
Be careful as once a sector is chosen it cannot be further subdivided. 

### Base System
```shell
ros2 launch stack_master base_system_launch.xml map_name:=<name of mapped track> sim:=<true/false> racecar_version:=<NUCX used>
```
  - `<name of mapped track>` is the name of the track you want to run on. It must belong to the list of maps available in the `stack_master/maps` folder or be omitted. Then the map called `latest` is used. 
  - `<true/false>` is a boolean value that indicates if you want to run the simulation or the real car. 
  - `<NUCX>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car). You can also omit this if the `.env` file was created.

### Time trials 
```shell
ros2 launch stack_master time_trials_launch.xml racecar_version:=<NUCx used> LU_table:=<Look-Up Table name> ctrl_algo:=<control algorithm> 
```
  - `<NUCx>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car). If you omit this the one set in base_system is used. 
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

### Simulation with Opponent
It is possible to add a steerable opponent to the simulation by changing the following parameter in [stack_master/config/SIM/sim.yaml](./config/SIM/sim.yaml):
```yaml
    # opponent parameters
    num_agent: 2

    # opp starting pose on map
    sx1: 2.0
    sy1: 0.5
    stheta1: 0.0
```
and then launching the the simulator normally, eg:
```shell
ros2 launch stack_master base_system_launch.xml map_name:=glc_ot_ez sim:=true
```

Then, using the `sim_opp` flag, time trials can be rerouted to control the opponent:
```shell
# launch the ego car first!
ros2 launch stack_master time_trials_launch.xml ctrl_algo:=PP

# then you can control the opponent
ros2 launch stack_master time_trials_launch.xml ctrl_algo:=PP sim_opp:=true
```

The opponent exposes the following topics:
```
# simulator (gym_bridge)
/opp_drive                         # AckermannDriveStamped, opponent drive commands
/opp_scan                          # LaserScan, opponent lidar (sees the ego car)
/opp_racecar/odom                  # Odometry, opponent odometry
/opp_racecar/pose                  # PoseStamped, opponent pose
/opp_racecar/opp_odom              # Odometry, ego odometry as seen by the opponent
/car_state/opp_odom                # Odometry, opponent odometry as seen by the ego
/goal_pose                         # PoseStamped, resets the opponent pose (2D Goal Pose in RViz)

# time trials stack with sim_opp:=true
/opp_racecar/frenet/odom           # Odometry, opponent frenet odometry
/opp_racecar/frenet/pose           # PoseStamped, opponent frenet pose
/opp_racecar/state                 # opponent state machine state
/opp_racecar/state_marker          # opponent state machine marker
/opp_racecar/local_waypoints       # opponent local waypoints
/opp_racecar/local_waypoints/markers
/opp_racecar/perception/obstacles  # opponent obstacle input
```
