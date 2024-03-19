# Spliner Tuning
# F1TENTH autonomous grand prix (ICRA'23 London) - ForzaETH

## Launching in ICRA STACK
Launch headtohead with spliner: 
```
roslaunch stack_master headtohead.launch LU_table:=NUCX_hangar_pacejka overtake_mode:=spliner od_mode:=sami
```

After successful launch, you should see the following messages in the command line:
- `[OBS Spliner] Waiting for messages and services...`
- `[Planner] Dynamic Spline Server Launched...`
- `[OBS Spliner] Ready!`

## Dnyamic Reconfigurable State Machine Parameter Overview
Launch `rqt` and go into `dynamic_statemachine_server`
### Parameters
- `lateral_width_gb_m`: the distance in [m] at which the statemachine considers an obstacle to be blocking the global trajectory.
- `lateral_width_ot_m`: the distance in [m] at which the statemachine considers an obstacle to be blocking the overtaking trajectory.
- `splini_hyst_timer_sec`: hysterisis time in [s] from which the statemachine waits until an overtake state can be entered again.
- `splini_ttl` time to live in [s] that caches the last splines s.t. we finish driving the spline even when switching to trailing state.
- `ftg_speed_mps`: threshold speed in [m/s] at which FTG is triggered to overtake with FTG when in a trailing zone and we are blocked. 
- `ftg_timer_sec`: time to await in [s] until we enter FTG.
- `ftg_active`: flag to activate the FTG based overtake.


## Dnyamic Reconfigurable Spliner Parameter Overview
Launch `rqt` and go into `dynamic_spliner_tuner_node`
### Parameters
- `evasion_dist`: the apex distance of the spline in [m]. 
- `obs_traj_tresh`: the threshold in [m] at which spliner considers obstacles, orthogonal to the raceline.
- `spline_bound_mindist`: minimum distance in [m] that the spline must be distanced to the track boundaries. If a single point of the spline is closer to the boundaries than the treshold, then the spline is aborted.
- `pre_apex_distX`: spline point that is infront of the apex. The spline consists of 7 points in frenet space. the further the `distX` params are set, the smoother the trajectory.
- `post_apex_distX`: spline point that is behind of the apex. You might want to have an asymetric spline, to cut off the opponent.
- `kd_obs_pred`: the gain parameter at which we propagate the `d` of the opponent back to the raceline. 
- `fixed_pred_time`: the time in [s] at which we forward propagate the opponent linearly.

## Troubleshooting
### No spline found
Lower the `evasion_dist` and the `spline_bound_mindist` but keep in mind that collisions are more likely.
### Our opponent cuts our raceline
Increase the `lateral_width_gb_m` and `obs_traj_tresh`. Optionally you can also increase the motion prediction by increasing the propagation time `fixed_pred_time`. 
#### These three params are dependant on the opponents behaviour:
- **If the opponent has a similar raceline** as us in the overtaking zones, then the params are not that important. 
- **If it is parallel to our raceline** the `lateral_width_gb_m` and `obs_traj_tresh` should be larger than the distance of the racelines.
- **If it cuts our raceline in a shallow angle** the `lateral_width_gb_m` and `obs_traj_tresh` should be chosen very large as well as higher motion propagation `fixed_pred_time`. **However** if we assume the opponent will overshoot after cutting our raceline, then a low `lateral_width_gb_m` will let us go into global trajectory tracking state quickly (once the opponent has overshot).
### In a non overtaking zone with an obstacle we are trapped
- Enable `ftg` in statemachine
### We don't seem to be using spliner waypoints fast enough
- Set down the `splini_hyst_timer_sec`
### Ghost splines come up
- When we have a very short circuit, splines of the previous lap can still be cached. Hence set down `splini_ttl`.

---
[Go back to the checklist index](./README.md)