# Perception: opponent and obstacle detection & tracking

Two nodes, launched by `stack_master/launch/head_to_head_launch.xml` via `launch/perception_launch.xml`:

| Node | Input | Output |
|---|---|---|
| `obstacle_detection` | `/scan`, TF, `/trackbounds/markers`, `/global_waypoints` | `/perception/detections` (`f110_msgs/DetectionArray`, internal) |
| `opponent_tracking` | `/perception/detections`, `/global_waypoints`, `/car_state/frenet/odom` | `/perception/obstacles` (`f110_msgs/ObstacleArray`) |

The logic lives in `obstacle_detection.py` / `opponent_tracking.py` (pure Python, no ROS), the ROS wrappers in the
`*_node.py` files, RViz output in `visualization.py`.

## Detection (`obstacle_detection`)
Per scan, every beam is transformed with its own laser pose (per-beam motion correction), and the scan is split into
segments at range jumps. A segment is kept only if *all* its points lie on the track and clear of the walls. The walls
are the planner's track bounds, corrected online by a learned per-bound offset (live wall model) for boards that were put
back slightly off. Kept segments are grouped into objects. For each object the node reports only what was actually seen
(`f110_msgs/Detection`): rear / front face and left / right edge (NaN if not seen or occluded), the heading from an
L-shape fit, and the bounding box of all its points. It makes no assumption about the object's size.

## Tracking (`opponent_tracking`)
A constant-velocity Kalman filter in Frenet coordinates per object; the state is the object's rear-centre. Each track
learns the width of the face it shows. Published to `/perception/obstacles`:
- **Cars** (moving, or not yet confirmed static): only if the observed face is car-like, i.e. at least ~12 cm and not
  wider than a car. The rules require every car to show a >= 12 x 12 cm square at 10-30 cm height. The centre is placed
  half a nominal car ahead of the rear, along the car's heading; the extent comes from the car size rules
  (`car_length_range`, `car_width_range`).
- **Static objects of any size**: once still for `static_min_time` (0.75 s), or `static_fast_time` (0.3 s) if they are
  less than `fast_range` ahead and overlap our path (+-`path_half_width` around the raceline or our current d).
  Published with the accumulated bounding box of everything seen of them.

False positives are treated as the worst failure: an object is only published after confirmation (3 of 4 scans), and
either a car-like shape or a confirmed standstill.

## Parameters
All parameters, with defaults and descriptions: `config/obstacle_detection_parameters.yaml` and
`config/opponent_tracking_parameters.yaml` (generate_parameter_library, read-only). Per-car overrides:
`stack_master/config/perception_params.yaml` and `stack_master/config/SIM/perception_params.yaml`.
