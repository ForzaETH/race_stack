#! /bin/bash
set -eo pipefail

# On x86 take torch from the CPU index, so pip does not pull the CUDA wheels.
# torch>=2.11 requires a setuptools newer than the 58.2.0 that ROS humble needs.
if [[ "$(uname -m)" == "x86_64" ]]; then TORCH_INDEX="--index-url https://download.pytorch.org/whl/cpu"; fi
pip3 install "torch<2.11" $TORCH_INDEX
