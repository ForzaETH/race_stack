# MPC
A suggested visualization tool for MPC is plotjuggler 

## Visualization-Test

Install plotjuggler:

```
sudo snap install plotjuggler-ros
```

Start-test:

```
plotjuggler-ros
```

## MPC Visualization

Step 1: start the simulator

```bash
# define the parameter of the simulator:stack_master/config/<racecar_version>/<racecar_version>_pacejka.yaml
roslaunch stack_master base_system.launch map_name:=f racecar_version:=<racecar_version> sim:=true tire_model:=pacejka
```
for example
```bash
roslaunch stack_master base_system.launch map_name:=f racecar_version:=NUC5 sim:=true tire_model:=pacejka
```

step 2: start the mpc controller

```bash
roslaunch stack_master time_trials.launch ctrl_algo:=STMPC
```

step 3: open plotjuggler 

```bash
plotjuggler-ros
# load xml file to visualization from:
<race_stack_folder>/controller/mpc/config/mpc_plot.xml
```

step 3: follow the command

## MPC Versions
Two MPC versions are available, one with a kinematic model, one with a single track, pacejka-tire model. To see the full list of parameters see, for example, [NUC5 configs](../../stack_master/config/NUC5/single_track_mpc_params.yaml), and to see which of those are dynamically reconfigurable, checkout the source code in the [(STMPC) config schema](../../stack_master/pbl_config/src/pbl_config/controller/mpc/STMPCConfigDyn.py).

### Usage Instructions
1. Start the simulator with the desired map and racecar version:
```bash
roslaunch stack_master base_system.launch map_name:=f racecar_version:=NUC5 sim:=true tire_model:=pacejka
```
2. Start the MPC controller with the time trials launch:
```bash
# for single-track mpc
roslaunch stack_master time_trials.launch ctrl_algo:=STMPC                          
```
or
```bash
# for kinematic mpc
roslaunch stack_master time_trials.launch ctrl_algo:=KMPC                          
```
**NOTE**: the parameters are implicitly chosen using the `racecar_version` and the `floor` in the `base_system.launch` step.
