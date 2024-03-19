#! /bin/bash

# install dep
apt-get install -y python3-wstool \
    python3-rosdep \
    ninja-build \
    stow \
    clang \
    cmake \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libceres-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libsuitesparse-dev \
    lsb-release \
    python3-sphinx \
    libgmock-dev \
    protobuf-compiler

# merge cartographer ws with ours
rm /etc/ros/rosdep/sources.list.d/20-default.list
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# install abseil 
./state_estimation/cartographer_pbl/cartographer/scripts/install_abseil.sh
