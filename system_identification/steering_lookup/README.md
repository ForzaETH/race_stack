# Steering lookup
This package provides an importable library to use the steering lookup obtained from sysid experiments.

## How to import python lib 

Build the library (catkin build steering_lookup) and source the workspace. Then you can import it in another ros package like so:

```python
from steering_lookup.lookup_steer_angle import LookupSteerAngle

# [...]

steer_lookup = LookupSteerAngle('NUC1_pacejka')
accel = 5.0 # m/s2
vel = 3.5   # m/s
steer_angle = steer_lookup.lookup_steer_angle(accel, vel)
# Output steer angle:
# rospy.loginfo(steer_angle)

```