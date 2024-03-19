# Frenet Odom Republisher

This nodes publishes the position and velocity of the racecar in frenet coordinates at the topic `/car_state/odom_frenet`.
The topic is of type `nav_msgs/Odometry`, and the specific Frenet information is encoded in this way:
  - s, or advancemente on the reference line is in `pose.pose.position.x`
  - d, or lateral deviation from the reference line is in `pose.pose.position.y`
  - longitudinal velocity is in `twist.twist.linear.x`
  - lateral velocity is in `twist.twist.linear.y`

NOTE: yaw is currently NOT relative to the trajectory

---
[Go back to the utils list](../../README.md)