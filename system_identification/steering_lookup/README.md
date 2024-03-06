# Steering lookup
This package provides an importable library to use the steering lookup obtained from sysid experiments for the MAP controller.

## How to use

Build the library (catkin build steering_lookup) and source the workspace. After generating the lookup table using the scripts in 'id_analyser', you can import this library and use it in another ros package like so:

```python
from steering_lookup.lookup_steer_angle import LookupSteerAngle

# [...]

steer_lookup = LookupSteerAngle('<NAME>')
accel = 5.0 # m/s2
vel = 3.5   # m/s
steer_angle = steer_lookup.lookup_steer_angle(accel, vel)
# Output steer angle:
# rospy.loginfo(steer_angle)

```

Replace \<NAME\> with the name of your config without "_lookup_table.csv". So if your config is called CAR2_pacejka_lookup_table.csv, \<NAME\> would be CAR2_pecejka.