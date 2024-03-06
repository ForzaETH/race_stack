#! /bin/bash

# Script to launch the main docker instance for the pblf110 car

docker exec --tty \
    --interactive \
    forzaeth_devcontainer \
    /bin/bash
