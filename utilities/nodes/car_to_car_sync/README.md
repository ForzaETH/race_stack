# car_to_car_sync

Streams the opponent car's state into the ego car and publishes it as a ground-truth
`ObstacleArray`, so overtaking can be tested without perception.

Cars keep their own `ROS_DOMAIN_ID`. Only the topics listed in `config/topics.yaml`
cross the Wi-Fi link: zenoh forwards a topic between routers only when the far side
subscribes to it.

## Setup

1. Each car has its own domain id (`remote_car.sh` convention: `40 + NUC number`).
2. On the **ego** car start the zenoh router peered with the opponent router:
   ```bash
   ./stack_master/scripts/run_zenoh.sh <OPP_IP>
   ```
3. Launch the ego stack **without** perception, then:
   ```bash
   ros2 launch car_to_car_sync car_to_car_sync_launch.xml opp_domain:=46
   ```

## Topics

| ego topic | source |
|---|---|
| `/opp/car_state/frenet/odom` | opponent `/car_state/frenet/odom` |
| `/opp/car_state/odom` | opponent `/car_state/odom` |
| `/perception/obstacles` | built from `/opp/car_state/frenet/odom`, empty when the stream stops for >0.5 s |

Both cars must run the same `map_name`, otherwise `s`/`d` do not match.

## Nodes

- `domain_bridge`: opens a second rclpy context on `opp_domain`, discovers the message type
  of each listed topic from its publishers and republishes under `prefix` (default `/opp`).
- `opp_gt_publisher`: frenet odom -> `ObstacleArray` at `rate` Hz, obstacle `size` m.
