# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is the ForzaETH Race Stack ROS 2 (Humble) implementation for autonomous F1/10 racing. It's a complete autonomous racing system supporting both simulation and physical hardware (NUC-based race cars). The stack implements head-to-head racing with opponent detection, overtaking, and time trials.

**Important Notes:**
- This is the ROS 2 version and is less tested than the ROS 1 version
- Some features from the paper are missing: Bayesian Optimization, CPU Usage Measurements, Car2Car Syncing, SynPF integration, Scan Alignment, System Identification
- Development is primarily done within Docker containers
- The stack supports both x86 (NUC) and ARM (Jetson) platforms

## Architecture

The system follows a modular ROS 2 architecture with the following main components:

### Core Modules

1. **stack_master**: Central coordination and launch management
   - Contains all main launch files (mapping, time trials, head-to-head, base system)
   - Manages map storage in `stack_master/maps/`
   - Configuration files for different car versions (NUC2, NUC5, NUC6, NUC7, SIM)

2. **state_machine**: Behavior coordination (`state_machine/`)
   - Manages state transitions between GBTRACK (global path tracking), OVERTAKING, and TRAILING
   - Implemented in `state_machine.py` with transitions in `transitions.py` and state behaviors in `states.py`
   - Publishes to `/local_waypoints` which the controller consumes

3. **planner**: Trajectory planning system
   - **global_planner**: Generates optimal racing lines using TUM global trajectory optimization
   - **spline_planner** (local): Real-time obstacle avoidance via spline-based trajectory modification
   - Global planner runs during mapping, local planner runs during racing

4. **controller**: Low-level control (`controller/`)
   - Managed by `controller_manager.py`
   - Implements MAP (Model and Acceleration-based Pursuit), PP (Pure Pursuit), and FTG (Follow the Gap) controllers
   - L1 adaptive lookahead distance based on velocity
   - Includes trailing controller for maintaining distance behind opponents

5. **perception**: Opponent detection and tracking (`perception/`)
   - Opponent detection: Segments LiDAR scans into obstacles
   - Opponent tracking: Tracks and classifies obstacles as static/dynamic using Kalman filtering
   - Only active in head-to-head mode

6. **state_estimation**: Localization and velocity estimation (`state_estimation/`)
   - Google Cartographer for SLAM/localization
   - EKF (robot_localization package) for velocity fusion from VESC odometry and IMU
   - `carstate_node` aggregates state information

7. **system_identification**: Vehicle calibration (`system_identification/`)
   - Steering lookup tables in `steering_lookup/cfg/`
   - Maps desired lateral acceleration to actual steering angles

8. **sensors**: Hardware interfaces (`sensors/vesc/`)
   - VESC motor controller interface
   - Converts between Ackermann commands and VESC inputs

9. **utilities**: Helper nodes and libraries
   - `f110_msgs`: Custom ROS message definitions
   - `frenet_conversion`: Coordinate transformations between Cartesian and Frenet frames
   - Utility nodes: map_editor, lap_analyser, opponent_publisher, sector_tuner, slam_tuner

10. **base_system**: Simulation and base hardware support
    - `f110_simulator`: f1tenth_gym integration for simulation
    - `f1tenth_system`: Teleop tools, Ackermann mux, base F1/10 functionality

### Data Flow

The typical data flow during racing:
1. Sensors (LiDAR, VESC, IMU) → State Estimation (Cartographer + EKF)
2. State Estimation → Perception (opponent detection/tracking) & State Machine
3. Global Planner (pre-computed) → State Machine
4. Perception + Global Path → Local Planner (Spliner) → State Machine
5. State Machine → Controller (publishes `/local_waypoints`)
6. Controller → VESC (publishes `/drive` commands)

### Coordinate Systems

- **Cartesian (map frame)**: Global reference, published on `/car_state/pose` and `/car_state/odom`
- **Frenet frame**: Curvilinear coordinates (s, d) along the raceline, published on `/car_state/frenet/odom`
  - s: progress along centerline
  - d: lateral deviation from centerline
  - Critical for trajectory planning and obstacle handling

## Development Environment

### Docker Setup

**Building the container:**
```bash
# Set environment variables
export UID=$(id -u)
export GID=$(id -g)

# For x86 platforms (NUC)
docker compose build nuc

# For ARM platforms (Jetson)
docker compose build jet
```

**Container cache structure** (must be created at `../cache/` relative to race_stack):
```
cache/
└── humble/
    ├── build/
    ├── install/
    └── log/
```

**Launching the container:**
```bash
# Update FORZAETH_DIR in .docker_utils/main_dock.sh to match your race_stack path
# Update IMAGE variable for your platform (nuc_forzaeth_racestack_ros2 or jet_forzaeth_racestack_ros2)

# Setup X forwarding
source .devcontainer/xauth_setup.sh

# Launch main container
./.docker_utils/main_dock.sh

# Inside container, run post-create setup
cd ~/ws/src/race_stack
./.install_utils/post_create_command.sh
```

**Additional terminals:**
```bash
# Attach to running container
./.docker_utils/sec_dock.sh

# Reopen closed container
./.docker_utils/main_attach_dock.sh
```

### Building the Stack

Inside the Docker container:
```bash
# From workspace root
cd ~/ws

# Build all packages
colcon build --symlink-install

# Build specific package
colcon build --packages-select <package_name>

# Source the workspace
source install/setup.bash
```

The build artifacts are cached in `../cache/humble/` outside the container for persistence.

## Common Commands

### Mapping a New Track

```bash
ros2 launch stack_master mapping_launch.xml racecar_version:=<NUCX> map_name:=<map_name>
```
- `racecar_version`: NUC2, NUC5, NUC6, NUC7, or SIM
- `map_name`: No whitespace (convention: location_day_version, e.g., "hangar_12_v0")
- After completing a lap, GUI prompts for global raceline generation
- Sector selection GUI appears (selections are permanent, cannot be subdivided)
- Requires ROS resourcing after completion

### Running the Base System

```bash
ros2 launch stack_master base_system_launch.xml map_name:=<name> sim:=<true/false> racecar_version:=<NUCX>
```
- Maps must exist in `stack_master/maps/`
- Sets up localization, state estimation, and base hardware/simulation

### Time Trials

```bash
ros2 launch stack_master time_trials_launch.xml racecar_version:=<NUCX> LU_table:=<table_name> ctrl_algo:=<MAP/PP>
```
- `LU_table`: Lookup table from `system_identification/steering_lookup/cfg/`
- `ctrl_algo`: MAP or PP controller

### Head-to-Head Racing

```bash
ros2 launch stack_master head_to_head_launch.xml racecar_version:=<NUCX> LU_table:=<table_name> ctrl_algo:=<MAP/PP> overtake_mode:=spliner
```
- Enables perception, state machine with overtaking/trailing behaviors
- Currently only `spliner` overtake mode is supported

### Troubleshooting

**Simulator issues (no car/scans showing):**
```bash
source ~/ws/src/race_stack/.install_utils/f110_sim_setup.sh
```

**Joystick not working:**
```bash
sudo chmod 666 /dev/input/js0
sudo chmod 666 /dev/input/event*
```

## Package Management

All main packages are Python-based ROS 2 packages using `setup.py`:
- controller
- perception
- planner/global_planner
- planner/local_planners/spline_planner
- stack_master
- state_estimation
- state_machine
- system_identification/steering_lookup
- system_identification/nodes/id_controller

After modifying Python code, you typically don't need to rebuild if using `--symlink-install`, but configuration/launch file changes may require:
```bash
colcon build --packages-select <package_name>
source install/setup.bash
```

## Key Configuration Locations

- **Car-specific configs**: `stack_master/config/<racecar_version>/`
  - SLAM parameters: `slam/f110_2d.lua` (mapping) and `f110_2d_loc.lua` (localization)
  - Controller parameters, state machine parameters, etc.
- **Maps**: `stack_master/maps/<map_name>/` (contains waypoints, boundaries, occupancy grids)
- **Steering lookup tables**: `system_identification/steering_lookup/cfg/`
- **Launch files**: `stack_master/launch/` (main entry points)

## Important Topics

**State topics:**
- `/car_state/pose`: PoseStamped in map frame
- `/car_state/odom`: Odometry in map frame
- `/car_state/frenet/odom`: Odometry in Frenet frame

**Control flow:**
- `/global_waypoints`: Pre-computed optimal trajectory
- `/local_waypoints`: Active trajectory for controller (from state machine)
- `/drive`: Ackermann drive commands to VESC

**Perception:**
- `/scan`: LiDAR data
- `/perception/obstacles`: Detected and tracked obstacles

**State machine:**
- `/state_machine`: Current state as string (GBTRACK, OVERTAKING, TRAILING, etc.)

## Development Notes

- **Parameter tuning**: Many parameters are dynamically reconfigurable via `ros2 param set`
- **Frenet frame**: Most planning/perception logic operates in Frenet coordinates (s, d)
- **Sector-based tuning**: Tracks are divided into sectors for per-sector parameter optimization
- **Visualization**: Use RViz2 with marker topics for debugging (most nodes publish debug markers)
- **Rate-limited nodes**: All main nodes specify their own loop rates in configuration
- **Map editor**: Available for manual map boundary adjustments and sector re-selection

## Citation

If using this codebase for research, cite:
```
@article{baumann2024forzaeth,
  title={ForzaETH Race Stack—Scaled Autonomous Head-to-Head Racing on Fully Commercial Off-the-Shelf Hardware},
  author={Baumann, Nicolas and Ghignone, Edoardo and K{\"u}hne, Jonas and Bastuck, Niklas and Becker, Jonathan and Imholz, Nadine and Kr{\"a}nzlin, Tobias and Lim, Tian Yi and L{\"o}tscher, Michael and Schwarzenbach, Luca and others},
  journal={Journal of Field Robotics},
  year={2024},
  publisher={Wiley Online Library}
}
```
