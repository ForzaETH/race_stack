# Obstacle Detection and Opponent Tracker
# F1TENTH autonomous grand prix (ICRA'23 London) - ForzaETH

## Launching in ICRA STACK
Launch headtohead with spliner: 
```
roslaunch stack_master headtohead.launch LU_table:=NUCX_hangar_pacejka overtake_mode:=spliner od_mode:=sami
```

After successful launch, you should see the following messages in the command line:
- `[Tracking] Dynamic Tracker Server Launched...`
- `[Perception Detect]: Ready`
- `[Perception Classifier]: Ready`

## Dnyamic Reconfigurable Parameter Overview
Launch `rqt` and go into `dynamic_tracker_server`
### Tracking
- `ttl_dynamic`: the time to live of the dynamic prediction of the opponent. 
- `ratio_to_glob_path`: the ratio between the target velocity and the global velocity. Defines to which target speed the dynamic prediction will convert. If set to 1, target velocity is equal the raceline speed
- `ttl_static`: time to live of the static and not-classified objects. Be aware that the ttl of static obstacle only gets decreased if they are/should be visible
- `min_nb_meas`: minimum number of measurements of a obstacle before it gets classified as static or dynamic
- `dist_deletion`: distance under which an obstacle can be deleted (if it is visible)
- `dist_infront`: distance under which obstacles infront of the car get published
- `min_std`: standard deviation of the position under which obstacles get classified as static. Needs to be increased when using pf2
- `max_std`: standard deviation of the position over which obstacles get classified as dynamic. Needs to be increased when using pf2
- `vs_reset`: average velocity in s direction under which a dynamic obstacle gets reset to a static obstacle
- `aggro_multi`: a multiplier for the association distance for dynamic obstacles
- `debug_mode`: will enable some prints to the console for debugging
- `publish_static`: enable/disable publishing of static obstacles. May help when many FP are present
- `noMemoryMode`: will disable the memory mode of the tracker. Can help with thight curves and trailing

### Detect
- `min_obs_size`: minimum size of the point could in laser points. Can be used to filter out FP
- `max_obs_size`: maximum size of an object in meters. Can be used to filter out FP
- `max_viewing_distance`: the maximum distance the detection can see. Maximum allowed for the race is 10m
- `boundaries_inflation`: artificially increases the boundaries for detection. Can help to filter out FP along the boundaries.

## Troubleshooting
### Detection/Tracking not working
Did you build and sauce?
### Trailing gets stuck in corners due to FP
- Enable `noMemoryMode`
- Increase `boundaries_inflation`

### FP near the boundaries
- Increase `boundaries_inflation`
- Consider inflating the boundaries with [map editor](/f110_utils/nodes/map_editor/README.md)

### Dynamic Obstalce is too fast/slow
- change `ratio_to_glob_path`, 1 is raceline speed

### Opponent/Obstacles are sometimes not visible
- decrease `min_obs_size`
- increase `max_viewing_distance`

### Very big obstacles (FP)
- decrease `max_obs_size`

### Missclasification
- increase `min_nb_meas`
- increase `min_std` and `max_std`
- incrase/decrease `vs_reset`

### Obstacle is not deletet but clearly inside of visible range
- enable `debug_mode` and fix it _OR_ disable `publish_static`

## Parameter in yaml
This variables should not need to be touched during the race. Listed for completeness.
### Tracking
- `rate`: set to 40Hz
- `P_vs`: proportional gain for the s-velocity
- `P_d`: proportional gain for bringing back the d-position to the center
- `P_vd`: proportional gain for the d-velocity
- `measurment_var_s`: the variance of the measurment noise in the s direction
- `measurment_var_d`: the variance of the measurment noise in the d direction
- `measurment_var_vs`: the variance of the measurment noise in the s-velocity
- `measurment_var_vd`: the variance of the measurment noise in the d-velocity
- `process_var_vs`: the variance of the process velocity noise in the s direction
- `process_var_vd`: the variance of the process velocity noise in the d direction
- `max_dist`: maximal distance for association of the obstacles
- `var_pub`: obstacles with a bigger variance are not published
### Detect
- `rate`: set to 40Hz
- `lambda`: minimum reliable detection range in degrees
- `sigma`: standard deviation of the noise of the lidar ranges in meter
- `min_2_points_dist`: minimum distance between two points
