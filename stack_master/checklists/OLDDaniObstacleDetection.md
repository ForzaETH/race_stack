# F110 Perception - Obstacle Detection

## Launching in ICRA Stack
- Launch `precheck.launch` or `headtohead.launch`.

## Parameter Overview:
 - `mode`: set either to `safe` for obstacle detection within the whole trackbounds or to `aggro` for obstacle detection within a certain width (see `aggromodewidth`)
 - `aggromodewidth`: maximum difference in d-value of obstacle to local waypoints to be considered during aggro mode (imagine it as a tube along the local waypoints)
 - `trackboundsoffset`: offset of the trackbounds towards the middle of the track. used to filter out false-positive obstacles
 - `minheight`: minimum height of lidar scan dot
 - `pitchoffset`: offset in radians of the measured pitch.
 - `maxrange`: maximum distance from car to obstacle along global waypoints for it to be considered
 - `mincounts`: how many ticks object has to be seen to be published
 - `timetolive`: how many ticks object is published since the last time it was detected
 - `fixedsize`: fixed size of obstacles published
 - `maxdistance`: maximum distance of two centerpoints for them to be considered the same obstacle
 - `minobjectsize`: minimum size of an obstacle
 - `maxobjectsize`: maximum size of an obstacle
 - `trackboundsheight`: height of trackbound pipes

## Tuning and Fixing

### Procedure
1. Make sure you have a rosbag (of an unsatisfactory lap). Run `rosbag filter /path/to/rosbag.bag /path/to/new_rosbag.bag "topic != /obstacles_markerarray"` to create a rosbag without the detected obstacles.
2. Identify the problems occuring in the original rosbag:
   1. False Positives due to lidar pointing into ground
   2. False Positives due to map shift
   3. Not detecting objects
   4. Detecting duplicate objects
3. Skip to your problem below and tune the parameters a little.
4. Run the new rosbag and `objectdetect.py` from a launch file and check if obstacle detection improved.
5. Repeat step 3 & 4 until satisfied. If you're still getting a lot of false positives, change mode to `aggro`.

### False Positives due to lidar pointing into ground:
- increase `minheight` (towards zero). Value should be somewhere between `-0.1` and `0.0`. Default is `-0.05`.
- fix `pitchoffset`. On NUC3 it is approx. `-0.013962634` rads; on NUC4 it is approx. `-0.0157079633` rads.

### False Positives due to map shift
- increase `trackboundsoffset`. Value should be somewhere between `0.0` and `0.3`. Default is `0.1`.

### Not detecting objects
- increase `mincounts`. Default is `3`.
- adjust `minobjectsize` and/or `maxobjectsize`.

### Detecting duplicate objects
- increase `maxdistance`. Default is `0.4`.
