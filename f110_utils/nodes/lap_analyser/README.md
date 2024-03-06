# Lap Analyser
This package contains a couple of feature to analyze laps.

There are currently two main analyzers:
  - LapAnalyser
  - VelAnalyser

They are put together in the data_analyser, which can be launched with its launch file:
```
roslaunch lap_analyser data_analyser.launch
```

## Analyzer info
### Lap Analyzer
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

It can be launched with the `lap_analyser.launch`. 

The car distance to the track boundary is calculated inside `lap_analyser.py` . Each lap the minimum recorded distance is published as well in a topic called `/min_car_distance_to_boundary`. 
*Comment*: In the future it would make sense to integrate this into the `/lap_data` topic. The minimum car distance to the track boundary can be used as a measure of how likely is the car to crash/touch the boundary. This is currently used to optimize sector scalers and l1 parameter.

### VelAnalyser
It provides information about the current velocity of the car and the target velocity of the trajectory. The message type is `geometry_msgs/Point` and the topic is `velocity/current`. 
The target veolcity is published at the `velocity/trajectory` topic. 

This node also provides information about the current steering input and the target one, respectively at `steering_input` and `steering/trajectory`. The message type is the same the velocity one. 

### Longitudinal Analyser
It provides information about the Acceleration based Velocity Controller and the Trailing controller.
Every lap the average as well as the maximal Velocity error or Trailing error respectively.
It is launched with `longitudinal_analyser.launch`
## Plotting features

There are some basic plotting feature to obtain live plots of important measures for tuning.

### Prerequisites
Install plotjuggler and all the ros plugins with the single command:
```
sudo apt install ros-noetic-plotjuggler-ros
```

Run the base system and a controller as usual.


### Usage
  1. Launch the data analyser with `roslaunch lap_analyser data_analyser.launch`
  
  2. Launch the data plotter with `roslaunch lap_analyser data_plotter.launch`
    
  3. Press yes when the following popup appears (it appears twice):
    ![image.png](./data/image.png)

  4. When the "Select ROS messages" popup appears, check that the following topics are selected:
      - `/car_state/odom_frenet`
      - `/data_analyser/*` (should be five in total)
      - `/vesc/sensors/imu/raw`

      Then press ok
  5. Press OK on the warning popup that might appear. Two `plotjuggler` screens will appear. Most likely not all the curves will be correctly visualised.
  6. Reload the config files by pressing the "load data and windows/plot layout" button as in the picture 
    ![image-1.png](./data/image-1.png)

  and by choosing the files `config/lapplot_pj.xml`, `config/nadineplot_pj.xml` or `config/long_ana_pj.xml`, depending on the features you want do analyse

  1. now all the plots should be working. For a better visualization experience set the buffer of the nadine plot to the first integer value larger than the lap time and set the lap plot buffer to roughly ten times that value. Buffer is set up in the upper left of plot juggler:
    ![image-2.png](./data/image-2.png)


*Note* : This setup was only tested in simulation. 


    


