#!/bin/bash

# Define the base directory where maps are stored
BASE_DIR=~/ws/src/race_stack/stack_master/maps

# Check for the provided map name
if [ "$#" -ne 1 ]; then
    echo "No map name provided. Available maps:"
    ls $BASE_DIR
    exit 1
fi

MAP_NAME=$1
FILE_PATH="$BASE_DIR/$MAP_NAME/global_waypoints.json"

# Check if input file is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <path-to-json-file>"
    exit 1
fi

# Create a backup of the original file
cp "$FILE_PATH" "$FILE_PATH.bak"

# Check if jq is installed
if ! command -v jq &> /dev/null
then
    echo "jq could not be found, please install it."
    exit 1
fi

# Process the file
jq '
  def transform_stamp_lifetime: .secs as $secs | .nsecs as $nsecs | del(.secs, .nsecs) | .sec = $secs | .nanosec = $nsecs;

  (.. | objects | select(has("header"))).header |= (
    del(.seq) |
    .stamp |= transform_stamp_lifetime
  ) |
  (.. | objects | select(has("lifetime"))).lifetime |= transform_stamp_lifetime
' "$FILE_PATH" > temp.json && mv temp.json "$FILE_PATH"

echo "The 'seq' fields have been removed and 'stamp' and 'lifetime' fields adjusted in all relevant objects in $FILE_PATH"
