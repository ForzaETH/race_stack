# Map Editor

The Map Editor package exists as a utility to process mapping data before the [global planner](../../../planner/global_planner/README.md) is invoked. 

## Why does the map need to be manually edited?
The Global Planner uses a computational algorithm (a Voronoi distance transform) to find the perfect centerline of your racetrack. This algorithm **strictly requires a closed-loop environment**. If your real track has gaps (i.e. the LiDAR misses a section of a wall and sees infinite hallway space), the algorithm will fail to recognize the circuit geometry. We bypass this by manually dropping fake walls into the image.

Other reasons to manually edit the map include:
- The physical track is very crowded (in a F1TENTH competition) and there are many static obstacles that take up track space.
- There are temporary obstacles on the track that will not be there at competition-time.


---

## Step 1: Generate the Raw Map

First, you must drive the car around the physical space to gather the raw LiDAR scan data.

1. Launch the Map Editor in **Mapping Mode** (`map_editor_mapping:=True`). This explicitly bypasses the strict "1-lap completion" algorithmic check, letting you save the grid whenever you feel the coverage is good enough.
    ```bash
    ros2 launch map_editor map_editor_launch.xml map_name:=MAP_NAME map_editor_mapping:=True racecar_version:=NUCNUC
    ```

2. Manually drive the car around the full track using your controller until the map is sufficiently explored (clear trackbounds, good loop closures). You can stop anywhere.
3. Save the map and close the matplotlib window. This will save a baseline map (png, yaml) and pbstream file **on the Car** at the directory `race_stack/stack_master/maps/MAP_NAME/`.

## Step 2: Understand the Two-Map Architecture

If you navigate into your `stack_master/maps/MAP_NAME/` folder, you will notice two very distinct, highly important image files:
* **`pf_map.png`**: This is the **Localization Map**. It contains the raw, jagged gaps exactly as the LiDAR saw them. The car will use this purely to localize its physical position against reality while racing. **Never edit this file.** (Unless you really need to ensure the Particle Filter algorithm can properly localize against the real track!)
* **`MAP_NAME.png`**: This is the **Planning Map**. This is the file we will edit to trick the global planner into generating a perfect flawless racing line.

## Step 3: Patch the Gaps (Close the Loop!)

Your objective is to "photoshop" the Planning Map so that the algorithm sees a perfectly enclosed loop.

1. Open `stack_master/maps/MAP_NAME/MAP_NAME.png` in a basic image editor (like GIMP or Drawing).
2. Select the color **Black** (`#000000`) and choose the Pencil tool.
3. Look for gaps in the track bounds where the white space bleeds into the outside grey voids. Draw thick solid black lines across those gaps. **Make absolutely sure the inside "Driveable Space" is securely boxed in!**
4. Select the color **White** (`#FFFFFF`) and erase any "Ghost Obstacles".
5. Save the image and **overwrite** the original `MAP_NAME.png`. Do not rename it.

## Step 4: Generate the Final Global Waypoints

Now that you have a perfectly closed loop in your `MAP_NAME.png`, we feed it back into the Map Editor one last time to call the Global Planner.

1. Run the same launch command locally, but set mapping mode to **`False`**:
    ```bash
    ros2 launch map_editor map_editor_launch.xml map_name:=MAP_NAME map_editor_mapping:=False racecar_version:=NUC2
    ```
    *(If the global trajectory direction arrow is wrong, add `reverse:=True` to the command).*
2. Because mapping mode is off, the stack will ignore live LiDAR and instantly read your manually patched `MAP_NAME.png`. 
3. A window with the map will pop up. Inspect that it makes sense. Close that window to proceed.
4. Next, other windows will pop up prompting you to select the speed and overtaking sectors. After selecting the sectors for both, click "done" and close the windows.
5. The Map Editor will then create the `speed_scaler.yaml`, `ot_sectors.yaml`, and `global_waypoints.json` files in the `stack_master/maps/MAP_NAME/` directory.


You are now fully configured and ready to race! Verify by running the base system **on the car** with the new map.
