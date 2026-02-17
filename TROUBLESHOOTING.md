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