# Mapping & Global Waypoints Generation Guide

This guide walks you through the step-by-step process of creating a raw map, manually patching any imperfections (closing the track loop), and passing it back through the map editor to extract mathematically perfect global waypoints for racing.

## Why does the map need to be manually edited?
The Global Planner uses a computational algorithm (a Voronoi distance transform) to find the perfect centerline of your racetrack. This algorithm **strictly requires a closed-loop environment**. If your real track has gaps (i.e. the LiDAR misses a section of a wall and sees infinite hallway space), the algorithm will fail to recognize the circuit geometry. We bypass this by manually dropping fake walls into the image.

---

## Step 1: Generate the Raw Map

First, you must drive the car around the physical space to gather the raw LiDAR scan data.

1. Launch the Map Editor in **Mapping Mode** (`map_editor_mapping:=True`). This explicitly bypasses the strict "1-lap completion" algorithmic check, letting you save the grid whenever you feel the coverage is good enough.
   ```bash
   ros2 launch map_editor map_editor_launch.xml map_name:=MY_TRACK map_editor_mapping:=True racecar_version:=NUC2
   ```
2. Manually drive the car around the full track using your controller until you are satisfied with the wall coverage.
3. Click the **Save Map** button located in the Map Editor GUI (or execute the ROS2 save map service call in your terminal). 
4. The system will output the new map data into the directory `stack_master/maps/MY_TRACK/`. 

## Step 2: Understand the Two-Map Architecture
If you navigate into your `stack_master/maps/MY_TRACK/` folder, you will notice two very distinct, highly important image files:
* **`pf_map.png`**: This is the **Localization Map**. It contains the raw, jagged gaps exactly as the LiDAR saw them. The car will use this purely to localize its physical position against reality while racing. **Never edit this file.**
* **`map.png`**: This is the **Planning Map**. This is the file we will edit to trick the global planner into generating a perfect flawless racing line.

## Step 3: Patch the Gaps (Close the Loop!)

Your objective is to "photoshop" the Planning Map so that the algorithm sees a perfectly enclosed loop.

1. Open `stack_master/maps/MY_TRACK/map.png` in a basic image editor.
2. Select the color **Black** (`#000000`) and choose the Pencil tool.
3. Look for gaps in the track bounds where the white space bleeds into the outside grey voids. Draw thick solid black lines across those gaps. **Make absolutely sure the inside "Driveable Space" is securely boxed in!**
4. Select the color **White** (`#FFFFFF`) and erase any "Ghost Obstacles".
5. Save the image and overwrite the original `map.png`.

## Step 4: Generate the Final Global Waypoints

Now that you have a perfectly closed loop in your `map.png`, we feed it back into the Map Editor one last time.

1. Run the exact same launch command as Step 1, but this time set mapping mode to **`False`**:
   ```bash
   ros2 launch map_editor map_editor_launch.xml map_name:=MY_TRACK map_editor_mapping:=False racecar_version:=NUC2
   ```
2. Because mapping mode is off, the stack will ignore live LiDAR. Instead, it instantly reads your manually patched `map.png`. Running the Voronoi algorithm, it easily finds the exact centerline skeleton between your newly solid black walls, and drops waypoints evenly across the track's spine.
3. The Map Editor terminates, and your final raceline is automatically saved into your `stack_master/maps/MY_TRACK/` directory as **`global_waypoints.json`**. 

You are now fully configured and ready to race!
