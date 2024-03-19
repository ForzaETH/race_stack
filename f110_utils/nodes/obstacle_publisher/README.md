# Obstacle Publisher
This package currently publishes a dummy dynamic obstacle, in the shape of a ball, that can run at different speed profiles on different racelines.

It can be launched with the following command:
```
roslaunch obstacle_publisher obstacle_publisher.launch
```
## Parameters
The following parameters are currently available fir the `launch` file:
- `speed`: select the speed, in m/s, of the dummy obstacle. Ideal values are in the range [0, 20].
- `speed_type`: select the type of speed profile to follor. Available options are: 
  - `const` for a constant speed of `speed`
  - `raceline` to take the speed of the global waypoints scaled down by the parameter `speed_scaler`
  - `sine` speed profile with the formula `speed` $+ 3 * sin(0.3 * 2 \pi t)$
  - `step` stand still for 2.5s then drive with velocity = `speed` for 2.5s
- `trajectory`: select the trajectory among the available ones. Currently the available trajectories are `centerline/min_curv/shortest_path`. In the future we are planning to  integrate the `min_time` trajectory too.
- `start_s`: select the initial starting coordinate along the `s` Frenet coordinate.

## Examples
- Obstacle with global trajectory speed * 0.5: `roslaunch obstacle_publisher obstacle_publisher.launch speed_type:=raceline speed_scaler:=0.5`
- Obstacle with sine speed arount $3m/s$: `roslaunch obstacle_publisher obstacle_publisher.launch speed_type:=sine speed:=3`


## TODOs:
- integrate `min_time` trajectory
- integrate `start_d` too

## Video
Some example video
![dummy_obs_mul_traj](/uploads/997dad7513b8b72ac7f5f0b369e43c91/dummy_obs_mul_traj.m4v)


# Obstacle Collision Detector
This package is used to detect collisions between the ego vehicle and the obstacles. Detection logic is based on the Secret Sauce.

To launch:
`rosrun obstacle_publisher collision_detector.py`

Inputs:
```
rospy.Subscriber("/obstacles", ObstacleArray, self.od_cb)
rospy.Subscriber("/car_state/odom_frenet", Odometry, self.odom_cb)
```

Outputs:
```
# True if collision detected, else False
self.col_pub = rospy.Publisher("/opponent_collision", Bool, queue_size=10)
# Minimal distance to an obstacle in Float32
self.opp_dist_pub = rospy.Publisher("/opponent_dist", Float32, queue_size=10)
```

Secret Sauce:
```
def collision_check(self, obs_arr: ObstacleArray, car_odom: Odometry):
        dists = [69.69]
        for obs in obs_arr.obstacles:
            car_s = car_odom.pose.pose.position.x
            car_d = car_odom.pose.pose.position.y

            od_s = obs.s_center
            od_d = obs.d_center

            # check distance to obstacle
            dist = ((car_s - od_s)**2 + (car_d - od_d)**2)**0.5
            dists.append(dist)

            if abs(car_s - od_s) < 0.25 and abs(car_d - od_d) < 0.15:
                return True, dist
        # no collision
        return False, min(dists)
```

---
[Go back to the utils list](../../README.md)