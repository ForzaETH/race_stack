# Usage of the different Scripts

## Convert Old Wpnts
If you want to use old racelines from the ROS1 Racestack, the ROS2 messages need to be adapted, use this script to apply this. This script transforms `global_waypoints.json` files within map directories located at `~/ws/data/maps`. It modifies `header` and `lifetime` objects in the JSON structure.

## Prerequisites
- `jq` must be installed.

## Usage
Run the script with the name of the map directory:
```bash
./convert_old_gbwpnts.sh <map_name>
```

## Example
```bash
./convert_old_gbwpnts.sh GLC_smile_small
```

## Figure out which maps are there
This will list the maps in the `~/ws/data/maps` directory:
```bash
./convert_old_gbwpnts.sh
```