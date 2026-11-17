#!/bin/bash

ros2 bag record -a -x '/camera/.*/(compressed|compressedDepth|theora)' -o $HOME/ws/src/race_stack/bags/$1