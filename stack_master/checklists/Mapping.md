
# Checklist for mapping phase (with map editor)
# F1TENTH autonomous grand prix (ICRA'23 London) - ForzaETH


## Roles
- **Mapping_MissionControl:** [Michael / Tobias]
- **Control_MissionControl:** [Luca / Filippo]


## Procedure 

## Precheck:
- Check the correct branch is used (`master`)
- Fully charged Battery plugged in
- Controller sufficiently charged and laying next to launch pc
- No wires in the way, battery is firmly positioned
- Car placed at a good position to start mapping (e.g. in the center of the track)
- Chosen NUC and controller are turned on
- Check that everybody is at their correct position:  
   [Mapping_MissionControl]: at the launch pc  
   [Control_MissionControl]: at the launch pc
- Launch pc connected to a secondary display (e.g. monitor)
- In the config file `stack_master/config/global_planner_params.yaml`, check:
   The `safety_width` and `safety_width_sp` (should be at least `0.7`)


## Setup:
1. Open a terminal and ssh into the car. `catkin build` and `sauce`
2. Open a terminal on your local pc. Open rviz and move it on the secondary display
   ```
   pit_rvizX
   ```

## Map with Mapping [Mapping_MissionControl]
If you have a clear racetrack, you can map with mapping
1. Launch mapping with:
   ```
   roslaunch stack_master mapping.launch racecar_version:=NUCX map_name:=PLACE_DAYMONTH_VN
   ```
   - Replace `PLACE` with the place where the track is, e.g. `icra` or `hangar`
   - Replace `DAYMONTH` with the zero-padded current day, e.g. `2805` if it is the 28th of may
   - Replace `VN` with the version of the map, e.g. `v0` if it is the first version of the map on that day
2. Drive at least one lap, using the automatic driving button (R1).
3. After one completed lap you will be asked if the map is good enough. 
   - Press `y` if the map looks good (clear trackbounds, no shifts)
   - Check if the waypoints were computed and the markers of the trackbounds look fine. Otherwise, kill everything and start over 
   from 1.

**Note: if the car does not prompt you to press `y` maybe the loop closure failed. Drive another lap manually**

4. If it does not work, use Map Editor


## Map with Map_Editor [Mapping_MissionControl]
If you have problems with getting a correct map (gaps in the racetrack or other cars on the map) or want to adjust the raceline, you can use [Map Editor](/f110_utils/nodes/map_editor/README.md). Prefer using Map Editor on our local machine, especially if the WLAN connection is bad.


## Create global trajectory [Control_MissionControl]
Depending on whether you chose to use map editor locally or on the car, create the global trajectory according to the [Readme](/f110_utils/nodes/map_editor/README.md).

You might want to make edits to the map to create specific racelines. However, remember that the `pf_map` should reflect the actual track, while the raceline-specific map can be edited as you wish.

1. Slice the map into sectors with the GUI.
2. Determine the overtaking sectors with the other GUI.
3. **Source all terminals freshly!** 

## Copying maps / Renaming maps [Control_MissionControl]
1. Rename the enclosing map folder from OLDNAME to NEWNAME.
2. Inside the folder, rename: `OLDNAME.pbstream` to `NEWNAME.pbstream`
3. Inside the folder, rename: `OLDNAME.png` to `NEWNAME.png`
4. Inside the folder, rename: `OLDNAME.yaml` to `NEWNAME.yaml`
5. Open up the `NEWNAME.yaml` and inside replace the reference to `OLDNAME.png` to `NEWNAME.png`


## Verification (on car)
Verify that the map was created correctly by starting the base system with the new map and check if it runs without errors
```
roslaunch stack_master base_system.launch racecar_version:=<VERSION> map_name:=<map_name>
```

Congrats, you are done with mapping
