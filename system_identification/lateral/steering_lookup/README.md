# Steering lookup
This package provides an importable library to use the steering lookup obtained from sysid experiments. The table is read through `pbl_config` from `stack_master/config/<racecar_version>/LUT/<floor>/default.csv`; generate it with `ros2 run lateral_sysid_ros generate_lut`.

## How to import python lib

Build the library (`colcon build --packages-select steering_lookup`) and source the workspace. Then you can import it in another ros package like so:

```python
from steering_lookup.lookup_steer_angle import LookupSteerAngle

# [...]

steer_lookup = LookupSteerAngle('NUC6', 'glc', self.get_logger().info)
accel = 5.0 # m/s2
vel = 3.5   # m/s
steer_angle = steer_lookup.lookup_steer_angle(accel, vel)
```
