#!/bin/bash

# Define the acados installation path
ACADOS_ROOT=${1:-/acados}

# Clone acados into the target folder
git clone https://github.com/acados/acados.git $ACADOS_ROOT
cd $ACADOS_ROOT
git submodule update --init --recursive

# Build and install acados
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON .. # Add more options as needed
make install -j4

# Update shared library path
make shared_library

# Install Python interface
pip install $ACADOS_ROOT/interfaces/acados_template

# Add environment variables to .bashrc and/or .zshrc
echo "export ACADOS_SOURCE_DIR=$ACADOS_ROOT" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$ACADOS_ROOT/lib" >> ~/.bashrc
