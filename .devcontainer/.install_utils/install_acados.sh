#!/bin/bash

# Define the acados installation path
ACADOS_ROOT=${1:-/acados}

# Clone acados into the target folder
git clone --branch v0.4.4 https://github.com/acados/acados.git "$ACADOS_ROOT"
cd "$ACADOS_ROOT" || exit
git submodule update --init --recursive

# Build and install acados
mkdir -p build
cd build || exit
cmake -DACADOS_WITH_QPOASES=ON .. # Add more options as needed
make install -j4

# Update shared library path
make shared_library

# Install Python interface
pip install "$ACADOS_ROOT"/interfaces/acados_template

# Add environment variables to .bashrc and/or .zshrc
echo "export ACADOS_SOURCE_DIR=$ACADOS_ROOT" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$ACADOS_ROOT/lib" >> ~/.bashrc

# Manually install older version of tera_renderer because ubuntu 20.04 (needed for Noetic) doesn't have glibc >= 2.32
wget https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux -O $ACADOS_ROOT/bin/t_renderer
chmod +x $ACADOS_ROOT/bin/t_renderer

echo "Acados installation completed."
echo "Run 'source ~/.bashrc' or 'source ~/.zshrc' depending on your shell."
