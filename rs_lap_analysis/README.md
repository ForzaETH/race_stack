# rs_lap_analysis
Lap and sector statistics of the car. `lap_analyser` is started by `stack_master/launch/base_system_launch.yaml`.

## Lap Analyser
Splits every lap into the speed sectors and publishes, per sector and per lap: time, mean/max lateral error to the
raceline, closest approach to a wall and mean/max speed.

Subscribes to:
- `/car_state/frenet` (`rs_msgs/FrenetState`): position along the raceline, all timing uses its stamps
- `/global_waypoints` (`rs_msgs/WpntArray`, latched): raceline and its distances to the walls
- `/sectors` (`rs_msgs/SectorArray`, latched, from `global_plan_publisher`): always at least one sector

Publishes:
- `/lap_analyser/sector_data` (`rs_msgs/SectorData`): when the car leaves a sector
- `/lap_analyser/lap_data` (`rs_msgs/LapData`, latched): at the finish line, with the lap's `SectorData` in `sectors`

The partial lap the node starts in is never published as a lap, only its sectors driven from their start are.
