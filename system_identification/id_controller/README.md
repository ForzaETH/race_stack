# Id controller
This package contains the node to run the different system identification experiments. Some of these require disabling or circumventing safety checks. Make sure you understand what you are doing before running anything. It's generally best to follow the System Identification procedure outlined [`here`](../../stack_master/checklists/SysID.md).

## Basic usage

The `id_controller` node sends the commands corresponding to the experiment to the car

The node is launched via
```shell
roslaunch id_controller id_controller.launch
```

The following arguments are available
  - `experiment` : indicates the number of the experiment (1 to 5 are available)
  - `id_param_file` : indicates the postion of the config file for the experiments. Default should be fine for most cases, If you need to define a new one, look at the default [here](./parameters/experiments.yaml)
  - `drive_topic` : indicates where the topics for commanding the car will be published. Default should be fine in all cases for now.

## Experiments Types
There are currently 5 types of experiment available
### **1**:
find the parameters for https://git.ee.ethz.ch/pbl/research/f1tenth/race_stack/-/blob/master/sensors/vesc/vesc_ackermann/src/ackermann_to_vesc.cpp
Namely the steering_to_servo_gain_, steering_to_servo_offset_

### **1** Steering Angle to Servo Mapping

Namely the steering_to_servo_gain, steering_to_servo_offset, required to map steering angle commands to actual commands sent to the servos.

For this, we bridge the ackermann_to_vesc node and directly publish onto the vesc topics via.

```cpp
  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/speed", 10);
  servo_pub_ = nh.advertise<std_msgs::Float64>("commands/servo/position", 10);
```

# BE CAREFUL
This only works if no other node publishes onto these topics. As this would require deactivating the safety switches in place, it's preferable to use the iterative method using experiment 5, described in [`the checklist`](../../stack_master/checklists/SysID.md).

Parameters are set in the [config file](./parameters/experiments.yaml).
```python
## Experiment 1 ##

# Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
# multiplied by the number of motor poles. Any value is accepted by this
# driver. However, note that the VESC may impose a more restrictive bounds on the
# range depending on its configuration.
const_erpm: 3000

# Commanded VESC servo output position. Valid range is 0 to 1., check out vesc driver and vesc.yaml: servo_min: 0.1, servo_max: 0.85
const_servo: 0.415
```

### **2**: Acceleration to Current Mapping
DO NOT USE
# BE CAREFUL
This experiment can go very wrong. It is only needed if you want to control the car via current commands, to control acceleration, as acceleration can otherwise not be commanded directly. It is however strongly advised to command motor speeds for safety reasons. 

Again, to command current directly to the motors, it need to be ensured that no other nodes are publishing to "/vesc/commands/motor/speed" or "/vesc/commands/motor/current". This again requires disabling/killing the safety switches, which is strong discouraged. 

The experiment commands a constant current `const_curr` for a defined `accel_time` and then a constant breaking current `const_brake` for a defined `decel_time`. Do not kill the node while the car is driving fast, otherwise it will not break. Triple check that your times are set correctly. From experience the maximum currents are around 60A. Anything more does not result in increased acceleration.

Directly relate acceleration of car to commanded motor current, accelerate and decelerate and record acceleration vs cmd current
```
a = C_curr_accel/m * I_cmd if I_cmd > 0
a = C_curr_decel/m * I_cmd if I_cmd < 0
```

Parameters are set in the [config file](./parameters/experiments.yaml).
```python
## Experiment 2, send direct current commands and look at acceleration ##
#from vesc.yaml: current_min: 0.0, current_max: 100.0
const_curr: 40
const_brake: 50

accel_time: 1 # never exceed 2s
decel_time: 1 # never exceed 2s
```

### **3**: 
To make the acceleration identification safer, this removes the need of experiment 2, to circumvent safety check, by commanding acceleration via the drive topic specified in the launch file. By setting the jerk to 512, the `custom_ackermann_to_vesc_node` knows to interpret this as an acceleration command. It uses the currently estimated acceleration_to_current_gain_ and velocity_to_current_gain_, defined in the vesc.yaml.

Parameters are set in the [config file](./parameters/experiments.yaml).
# BE CAREFUL
time parameters are set in experiment 2
```python
accel_time: 1 # never exceed 2s
decel_time: 1 # never exceed 2s

# Experiment 3 const accel decel with acceleration commands via drive message
const_accel: 5
const_decel: -5
```

### **4**:
Parameters are set in the [config file](./parameters/experiments.yaml).
```python
# Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
# multiplied by the number of motor poles. Any value is accepted by this
# driver. However, note that the VESC may impose a more restrictive bounds on the
# range depending on its configuration.
const_erpm: 3000
...
# experiment 4, increase angle from start_pos, to end_pos over angle_time
# drives with const_erpm defined above
angle_time: 20

start_pos: 0.15
end_pos: 0.85
```

### **5**:
Command a linearly changing steering input and constant velocity for a predefined amount of time.

Parameters are set in the [config file](./parameters/experiments.yaml).
```python
# experiment 5, increase steering angle, drive with const_speed
const_speed: 3

start_angle: 0.1 # rad
end_angle: 0.4    # rad
```
