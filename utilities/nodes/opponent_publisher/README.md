## Obstacle Publisher
This  publishes a dynamic obstacle that can run at different speed profiles on different racelines.

It can be launched with the following command:
```
ros2 launch opponent_publisher opponent_publisher_launch.xml 
```
### Parameters
The following parameters are currently available fir the `launch` file:
- `constant_speed`: True or False
- `speed_scaler`: if `constant_speed` is False the opponent will drive at this scaled raceline velocity, if `constant_speed` is True the opponent will drive at this speed
- `trajectory`: select the trajectory among the available ones. Currently the available trajectories are `centerline/min_curv/shortest_path`. In the future we are planning to  integrate the `min_time` trajectory too.
- `start_s`: select the initial starting coordinate along the `s` Frenet coordinate.
- `type`: select "virtual" or "lidar" and the opponent will either spawn as a dummy obstacle (already has perception) or as a lidar detectable point (needs to be detected).

### Examples
- Obstacle with global trajectory speed * 0.5 and Lidar Obstacle: `ros2 launch opponent_publisher opponent_publisher_launch.xml speed_scaler:=0.5 trajectory:=min_curv type:=lidar`
- Obstacle with constant speed at $3m/s$: `ros2 launch opponent_publisher opponent_publisher_launch.xml constant_speed:=True speed_scaler:=3.`


# Obstacle Collision Detector
This package is used to detect collisions between the ego vehicle and the obstacles. Detection logic is based on the Secret Sauce.

To launch:
`ros2 run opponent_publisher collision_detector`

Inputs:
```
Subscriber("perception/obstacles", ObstacleArray, self.od_cb)
Subscriber("/car_state/odom_frenet", Odometry, self.odom_cb)
```

Outputs:
```
# True if collision detected, else False
Publisher("/opponent_collision", Bool, queue_size=10)
# Minimal distance to an obstacle in Float32
Publisher("/opponent_dist", Float32, queue_size=10)
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
