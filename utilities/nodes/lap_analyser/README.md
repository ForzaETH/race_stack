# Lap Analyser
This package contains a feature to analyse laps.
## Analyzer info
### Lap Analyser
It provides information about completed laps.
Specifically it provides the information in the custom message `f110_msgs/LapData`on the topic `lap_data`.
The (self-explanatory) structure of the message is as follows:
```
std_msgs/Header header

int32 lap_count 
float64 lap_time
float64 average_lateral_error_to_global_waypoints
float64 max_lateral_error_to_global_waypoints
```

It can be launched with `lap_analyser_launch.xml`. 

The car distance to the track boundary is calculated inside `lap_analyser.py` . Each lap the minimum recorded distance is published as well in a topic called `/min_car_distance_to_boundary`. 
*Comment*: In the future it would make sense to integrate this into the `/lap_data` topic. The minimum car distance to the track boundary can be used as a measure of how likely is the car to crash/touch the boundary. This is currently used to optimize sector scalers and l1 parameter.