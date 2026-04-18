# Networking & Remote Connection Guide

## Zenoh remote connection
This guide explains how to connect your external laptop or desktop to the physical autonomous car so you can seamlessly view all live ROS2 communication, topics, and RViz data over Wi-Fi through Zenoh.

## Prerequisites
This workflow assumes:
* You are using the official Docker container provided in `.docker_utils`.
* Your container is strictly following our Zenoh architecture as ROS2 Middleware.
* The Zenoh router (`rmw_zenohd`) automatically runs in the background on your car's docker container (via the updated `.bashrc_forzaeth`).

## Remote Observation Workflow

Follow these exact steps from your **external laptop** to connect to the live car natively:

### 1. Update the Car's IP Address
Before connecting for the first time, you must tell your laptop what the car's physical IP address is on your local Wi-Fi network.

1. Open `stack_master/scripts/remote_car.sh`.
2. Locate the `machines` dictionary at the top of the file.
3. Update the entry to match your car's exact IP address. For example:
   ```bash
   declare -A machines=(
       [NUC1]="192.168.1.55"  # Substitute with your car's real IP!
   )
   ```

### 2. Connect the laptop to the Car via Zenoh
To start receiving ROS packets recursively on your laptop, you need to execute the connection script from within your Docker container. This automatically sets your laptop as a Zenoh client and tunnels traffic directly into the car's router.

1. Open your terminal on your laptop and launch your Docker container (e.g., `./.docker_utils/main_dock.sh`).
2. Run the shortcut alias to instantly navigate to the scripts folder, and source the connection script with your target car:
   ```bash
   scripz && source remote_car.sh NUC1
   ```
   *(Be sure to replace `NUC1` with whatever string you configured in the dictionary array).*

### 3. Verify ROS Domain ID Matches
When you run the script above, pay close attention to the printed terminal output. It will tell you what `ROS_DOMAIN_ID` it just configured your laptop to use.

**Crucial Step:** The `ROS_DOMAIN_ID` running on your laptop *must perfectly match* the `ROS_DOMAIN_ID` configured on the physical car. If they do not match, Zenoh will silently drop the communication. If you need to manually change it, you can edit the hardcoded `export ROS_DOMAIN_ID=X` line inside `remote_car.sh`.

### 4. Observe the Live Data!
Once the script is sourced, your laptop is fully integrated into the car's network. From the exact same terminal, you can immediately observe traffic:
- Try `ros2 topic list` to see all active topics from the car.
- Launch `rviz2` to view the live Map, Global Waypoints, and LiDAR scans happening on the car in real time.
