# Transform_Broadcaster

Ever need to broadcast a TF transform from an Odom frame?

Look no further than `transform_broadcaster`.

Simply adapts the [TF2 Broadcaster tutorial](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29) to our uses.

## Parameters
- `odom_topic`: What `nav_msgs/Odometry` topic are you subscribing to?
- `tf_child_frame`: What is the child frame? Defaults to `base_link`.

---
[Go back to the utils list](../../README.md)