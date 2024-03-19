# Frenet Conversion Server 

This is a `rosservice` that can be used to convert coordinates from the frenet frame (in terms of `\global_waypoints`) to global cartesian coordinates and viceversa.

To launch the service, run:
```
rosrun frenet_conversion_server frenet_conversion_server_node
```

To test the service from the command line, one can do 
```
rosservice call /convert_glob2frenetarr_service "x: 1.1 y: 2.1"
```
And the return message will be something like the following, depending on the `/global_trajectory`:
```
s: <some s>
d: <some d>
idx: <some index>
```

The other service available is `/convert_frenet2globarr_service`.

## Service structure 
  - Glob2FrenetArr
      ```
      # request
      float64[] x
      float64[] y
      ---
      # response
      float64[] s
      float64[] d
      int32[] idx
      ```
  - Frenet2GlobArr
      ```
      # request
      float64[] s
      float64[] d
      ---
      # response
      float64[] x
      float64[] y
      ```

## Example Python usage
An example of using the service can be found in `<race_stack folder>/perception/obstacle-detection/objectdetect.py`.

The service is imported in line 14:
```python
from frenet_conversion.srv import Glob2FrenetArr
```

Then the service proxy is initialised (lines 80-82):
```python
# --- Frenet Conversion Service ---
rospy.wait_for_service('convert_glob2frenetarr_service')
self.glob2frenet = rospy.ServiceProxy('convert_glob2frenetarr_service', Glob2FrenetArr)
```

And the proxy is used for instance in lines 170-173
```python
# Do frenet conversion from (x,y) [map] -> (s,d) [frenet wrt min curv] via conversion service where x, y, s and d are arrays of arbitrary length
resp = self.glob2frenet([x1, x2], [y1, y2])
s_points = resp.s
d_points = resp.d
```

---
[Go back to the utils list](../../README.md)
