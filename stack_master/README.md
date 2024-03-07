# Stack Master
Here is the `stack_master`, it is intended to be the main interface between the user and the PBL F110 system.

## Basic Usage
Here only quick procedures are detailed, for more details check the checklists [here](./checklists/)

### Mapping (on the real car)
It is suggested to first launch a roscore in a separate terminal. This will be useful to keep any RViz sessions working even across restarts of the base system.
```shell
roscore
```

Then launch the mapping procedure with the following launch file:
```shell
roslaunch stack_master mapping.launch map_name:=<map name of choice> racecar_version:=<NUCX>
```

  - `<map name of choice>` can be any name with no white space. Conventionally we use the location name (eg, 'hangar', 'ETZ', 'icra') followed by the day of the month followed by an incremental version number. For instance, `hangar_12_v0`.
  - `<NUCX>` depends on which car you are using. NUC2, JET1, etc.

After completing a lap, press the requested button and the raceline will be generated. Then two GUI will be shown, and within them a slider can be used to select the sectors.
Be careful as once a sector is chosen it cannot be further subdivided.

After both the standard sectors and the overtaking sectors are chosen, the mapping procedure will re-build the `sector_tuner` and the `overtaking_sector_tuner` package, and then after this is completed the procedure can be ended with Ctrl+C.

You will then need to re-source the ROS environment, which can be done as:
```shell
source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash
```

If you don't have `sauce` already set as an alias in your `bashrc`.

### Base System

The base system launches the common components that are required for the car between time-trials and Head-to-Head mode. It can be launched as follows:

```shell
roslaunch stack_master base_system.launch map_name:=<map name of choice> racecar_version:=NUCX
```

For more information consult the launchfile [here](./launch/base_system.launch).

> Note: There is duplicated code in `headtohead` and  `time_trials` launchfiles. For example:
>
> the `controller_manager` node
> 
> dynamic reconfigure for controller manager
>
> CPU measurement
>
> State machine

This could be moved to the base-system launchfile (unless I'm missing something?)

### Time trials
```shell
roslaunch stack_master time_trials.launch LU_table:=NUCX_hangar_pacejka
```

In a Time-Trials part of competition, or if you know you don't need to deal with dynamic obstacles, launch this.

### Head to Head
```shell
roslaunch stack_master headtohead.launch LU_table:=NUCX_hangar_pacejka overtake_mode:=<frenet/spliner>
```

When there are dynamic obstacles, run this.

## How to connect the pit
- On your pit laptop:
  ```shell
  cd <race_stack folder>/f110_utils/scripts/pit_starter
  source pit_starter.sh <YOUR_ZEROTIER_IP> <NUC3 or NUC4> rviz
  ```
- Enjoy freshly sourced NUCX and runs the rviz with the correct config.

Pro tips:
- If you only want to source the NUCX then dont use the `rviz` argument
- It helps to alias these functions with convenient names
