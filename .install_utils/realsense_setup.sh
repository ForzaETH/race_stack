#! /bin/bash
set -e

ARCH=$(dpkg --print-architecture)
if [ "$ARCH" != "amd64" ] && [ "$ARCH" != "arm64" ]; then
    echo "No librealsense packages published for $ARCH, skipping."
    exit 0
fi

mkdir -p /etc/apt/keyrings

# Download and dearmor
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
gpg --dearmor | tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
echo "deb [arch=${ARCH} signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
tee /etc/apt/sources.list.d/librealsense.list
apt-get update
apt-get -y install librealsense2-utils
apt-get -y install librealsense2-dev
apt-get -y install librealsense2-dbg

# librealsense2-dkms is only published in the amd64 index and builds a kernel
# module, so it is neither available nor useful on arm64.
if [ "$ARCH" = "amd64" ]; then
    apt-get -y install librealsense2-dkms
fi
