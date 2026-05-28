# TROUBLESHOOTING

### 1. Existing map is not working correctly
Verify that all sectors have at least two waypoints for example `start: 100` and `end: 100` is invalid

### 2. Vehicle drive motor is not receiving commands
Powercycle the Jetson and VESC: shutdown the Jetson, disconnect, turn off the power board, disconnect the battery. Then reconnet everything and it should work.

### 3. During mapping, a piece of the map is not showing up in the `matplotlib` occupancy grid GUI
The map needs to have a minimum width - verify this piece of the track is wide enough

### 4. Cannot delete `colcon build` files from inside the Docker container
You are not able to delete the colcon build files inside the Docker container, with error:

```bash
jetson@jetson-desktop:~/ws$ rm -rf build/ install/ log/
rm: cannot remove 'build/': Device or resource busy
rm: cannot remove 'install/': Device or resource busy
rm: cannot remove 'log/': Device or resource busy
```

This is because these are mounted onto the host with the Docker container, see `race_stack/.docker_utils/main_dock.sh` The solution is to only delete the build file contents using command:

```bash
rm -rf build/* install/* log/*
```

### 5. Display forwarding from Jetson Docker container to pit machine is failing

Display forwarding must be working from the Jetson Docker container to the pit machine, for example for mapping with `mapping_launch.xml` with the `matplotlib` pop-up. Sometimes this stops working, for example with an error like 

```
UserWarning: Matplotlib is currently using agg, which is a non-GUI backend, so cannot show the figure
```

This is because, for display forwarding to work, it must be forwarded from the Docker container to the host Jetson, and then across SSH to the pit computer. Details for authentication can be found in `main_dock.sh` and `xauth_setup.sh`.

When the display forwarding fails, there is an easy fix to attempt, and a more involved fix. Start with the easy fix.

**Option 1: Easy Fix - localhost DISPLAY change**
1. Find the current display using `echo $DISPLAY`. It should show `localhost:10.0`
2. If the result is different, for example `localhost:13.0`, run `export DISPLAY=localhost:10.0`

**Option 2: Harder Fix - Docker rebuild**
If Option 1 fails, recreate the Docker container; the Docker image can be kept as-is. To do so, follow these steps:

1. Exit the container and find the name of the current container with `docker ps -a` - the most recently opened is the one you probably want to delete, for example, with name `forzaeth_racestack_ros2_humble`
2. Remove it as follows, replacing the name if necessary:
    ```bash
    docker rm forzaeth_racestack_ros2_humble
    ```
3. Follow the steps in the Docker utils [README.md](../race_stack/.docker_utils/README.md) after the image creation, beginning with **Step 4/5** with 
    ```bash
    cd <race_stack folder>
    source .devcontainer/xauth_setup.sh
    ```
    Followed by:
    ```bash
    ./.docker_utils/main_dock.sh
    ```
    And finally:
    ```bash
    cd ~/ws/src/race_stack
    ./.install_utils/post_create_command.sh
    ```

The issue should be resolved now.

### 6. PS4 controller /joy topic not publishing despite /dev/input/js0 being detected
The joy_node (running inside the Docker container) uses SDL2, which reads from /dev/input/event* rather than /dev/input/js*. The event devices are owned by root:input with mode crw-rw----, so the user must be in the input group to read them. systemd-logind grants a temporary ACL on these devices to the user of the active graphical seat, which is why pairing may work when logged in at the desk but fail later over SSH or after the session changes — the ACL is session-bound and ephemeral. Verify the issue by running ros2 run joy joy_enumerate_devices inside the container; if the table is empty despite ls /dev/input/js* showing the controller and jstest /dev/input/js0 reading inputs correctly, this is the cause. Fix it permanently by adding the host user to the input group (not within the container):

```bash
sudo usermod -aG input $USER
```

Then fully log out and log back in (a new terminal in the existing session will not pick up the new group). Verify with groups | grep input. The container inherits the host user's GIDs, so no container-side change is needed provided /dev/input is bind-mounted or passed via --device in main_dock.sh. The same pattern applies to other device groups worth joining preemptively on a new machine: dialout (VESC, USB-serial), video (cameras), plugdev (general hotplug).

### 7. Car is mysteriously stuck in TRAILING state in h2h mode, despite no objects in front of it
Sometimes the car is launched in h2h mode and stays in TRAILING mode stationary despite having no objects in front of it. When this occurs, a common explanation is the LiDAR is picking up bits of the car behind it, and recognizes these as objects. To avoid this, visualize the LiDAR LaserScan in RViz, and move cables or standoffs out of the way until the scan shows no objects behind the car. 
