# Sector and MAP Parameter Optimizer

The purpose of this is to use bayesian optimization to get optimal MAP parameters and/or sector scalers. The optimizer script can be launched side by side with the base system and time trials/head to head and does the optimization of the MAP parameter/sector scaler automatically. After the optimization, the parameters are set with dynamic reconfigure and can be permanentely stored manually in the corresponding yaml file.

## Example Procedure 
1.  Edit parameter ranges: 
    
    `race_stack/stack_master/config/NUCX/BO_range.yaml`. 

2.  Start time trials (either in sim or real car by pressing R1 continuously)

3.  Run the BO server and the node 

For Sector Scaling optimization:
```
roslaunch param_optimizer racecar_version:=NUCX param_optimizer.launch opt_sector_scalers:=True
```

For L1 param optimization:
```
roslaunch param_optimizer racecar_version:=NUCX param_optimizer.launch opt_l1params:=True
```
4. Check the settings printed in the terminal

5. Wait until it is finished


## Launch file arguments
- `opt_l1params`: Bool, to optimize the l1 parameter 
- `opt_sector_scalers`: Bool, to optimize sector scalers

**Optional**
- `iterations`: How many iterations does the algorithm do to optimize parameters (default: 10)
- `laps_per_experiment`: Laps per iteration: e.g. lap criteria is averaged over a certain number of laps before calculating the cost (default: 1)
- `cost_lap_time_scaler`: Lap time's contribution to the cost (default: 1)
- `cost_crash_avoidance_scaler`: Minimum car distance to track boundary's contribution to the cost (default: 1)
- `cost_lat_err_scaler`: Average lateral error's contribution to the cost (default: 1)

### Trouble shooting

One way of quickly testing if the installation worked is by launching the server:

```
roslaunch bayesopt4ros bayesopt.launch
```

You should see the output of the server similar to this:

```[INFO] [1616593243.400756]: [BayesOptServer] Iteration 0: Ready to receive requests.```

If roslaunch fails, make sure that the node script is executable.

```chmod +x src/bayesopt4ros/nodes/bayesopt_node.py```

More information and examples can be found [here](https://intelligentcontrolsystems.github.io/bayesopt4ros/getting_started.html#your-own-workspace)

### Other requirements
You need to have already some reasonable/good l1 parameters/sector scalers. This is in theory not necessary but will speed up the process of finding very good parameters. These reasonable params will be needed to decide for a suitable **range**.

## How it works in more Detail
The relevant parameters are the l1 parameters (`t_clip_min`, `m_l1` and `q_l1`) and/or the sector scalers (depends on specific track). 

Bayesian Optimization tests different value combinations of these variables within a given **range**. The range should be chosen relatively conservative, as we do not want to test instable values. It is stored in a config file `~race_stack/stack_master/config/NUCX/BO_range`. 

After updating the l1 parameters the car drives 1 to 3 laps (depending on the setting) and a cost is calculated as described below

```cost = f(distance to track boundary, average lateral error, lap time)```

The cost is sent to the Bayesian Optimization server and new parameters are requested. After receiving new parameters, the same process is repeated.

It is suggested to optimize l1 parameters and sector scalers seperately and not simultaneously.

## How to Run Node
Run time trials or head to head and press R1. After the car started to move you can run:

```
roslaunch param_optimizer racecar_version:=JETx/NUCx param_optimizer.launch 
```

If you want to optimize the l1 parameter add `opt_sector_scalers:=True` and if you want to optimize the sector scalers add `opt_l1params:=True`. By default it will optimize with 10 iteration and do 1 lap per iteration (also called laps per experiment). You can change that by adding e.g. 


```
iterations:=5 laps_per_experiment:=1 opt_l1params:=True 
``` 

Whenever you think the car is driving badly or dangerously you can stop pressing R1. Wait until there is a printout in the terminal: `Lap takes too long, not reasonable parameters. Skip!`. After this is printed the l1 parameters are resetted and the car should finish the lap nicely until it tries out new parameters. If the ranges are chosen conservative enough this should not be a problem. 

# Optimization Steps


1.  Edit parameter ranges here: 
    
    `~race_stack/stack_master/config/NUCX/BO_range.yaml`. 

2.  Start time trials by pressing R1 continuously

3.  Run the node 

For Sector Scaling optimization:
```
roslaunch param_optimizer racecar_version:=NUCX param_optimizer.launch opt_sector_scalers:=True
```

For L1 param optimization:
```
roslaunch param_optimizer racecar_version:=NUCX param_optimizer.launch opt_l1params:=True
```
4. Check the settings printed in the terminal

5. Wait until it is finished

Whenever you think the car is very unstable or cuts corners too aggressively, stop pressing R1 and wait until a message in the terminal appears: `'Lap takes too long, not reasonable parameters. Skip! (Press R1)'` . The parameters will be resetted and the car finishes the lap safely if you press R1 again.

In the end save the new l1 parameter here: `stack_master/config/NUCX/l1_params.yaml`

Parameters can also be obtained reading the logs in `param_optimizer/logs`, in case they are temporarily lost.

### Optional

Experiment with the acquisition function: UCB (upper confidence bound) or EI (expected improvement). In general: UCB is more explorative, which makes sense if the range is larger whereas EI is more exploitative. 

If you don't want to optimize a certain l1 parameter, e.g. `t_clip_min`, than you can its range very close, e.g. `[0.8, 0.0801]`

Editing cost:
Focus more on lap time by setting the cost scaler from 1 (default) to e.g. 10:

```
cost_lap_time_scaler:=10 cost_crash_avoidance_scaler:=1 cost_lat_err_scaler:=1 
``` 

---
[Go back to the utils list](../../README.md)