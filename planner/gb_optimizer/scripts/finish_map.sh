#!/bin/bash

echo "Finish trajectory..."
rosservice call /finish_trajectory 0

echo "Save map under the name $1"
rosservice call /write_state "{filename: '$1', include_unfinished_submaps: "true"}"
