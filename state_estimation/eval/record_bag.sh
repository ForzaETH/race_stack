#!/bin/bash

ros2 bag record -a -x '/camera/.*/(compressed|compressedDepth|theora)' -o "$BAGS_DIR/$1"