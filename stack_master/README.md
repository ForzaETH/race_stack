# Stack Master
Here is the `stack_master`, it is intended to be the main interface between the user and the PBL ForzaETH F110 system.

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

After completing a lap, a GUI will popup and pressing the requested button will start the global raceline generation. 
Then two GUIs will be shown, and within them a slider can be used to select the sectors. 
Be careful as once a sector is chosen it cannot be further subdivided. 

A ROS resourcing will be needed from here on. 

### Base System
```shell
ros2 launch stack_master base_system_launch.xml map:=<name of mapped track> sim:=<true/fasle> racecar_version:=<NUCX used>
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
