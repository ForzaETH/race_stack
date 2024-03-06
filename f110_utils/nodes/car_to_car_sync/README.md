# car_to_car_sync Package

The `car_to_car_sync` package provides a solution for redirecting ROS topics from one robot to another, enabling seamless communication and data sharing between the two robots. This is particularly useful when you want to use one (PhD) car as GND Truth for the main car (Professor).

## Overview

The package contains a Python script, `car2car_redirect_topic.py`, which subscribes to specified topics on Robot B, and republishes the data to corresponding topics on Robot A.

## Configuration

1. Update the `topics_to_redirect.yaml` file located in the `config` directory to list the topics you want to redirect. The file should follow this format:

    ```yaml
    car2car_topics:
    - subname: "/car_state/odom"
        pubname: "/opp/car_state/odom"
        type: "nav_msgs/msg/Odometry"
    - subname: "/car_state/odom_frenet"
        pubname: "/opp/car_state/odom_frenet"
        type: "nav_msgs/msg/Odometry"
    ```

2. In the `launch/car_to_car_redirect.launch` file, update the `professor_car_ip` parameter to match the IP address of Robot A.

## Running the Node

1. On **Professor Robot**, make sure the `roscore` is running and the `rosbridge_server` is launched:

    ```bash
    roslaunch rosbridge_server rosbridge_websocket.launch
    ```

2. On **PhD Robot**, launch the `car_to_car_sync` node:

    ```bash
    roslaunch car_to_car_sync car_to_car_redirect.launch professor_car_ip:=<IP_OF_PROFESSOR_CAR>
    ```

3. The node will now start redirecting the topics as configured in the `topics_to_redirect.yaml` file.

## Shutting Down

To safely shut down the node, you can press `Ctrl+C` in the terminal where the node is running. The node has a shutdown hook to ensure that the ROS bridge connection is properly terminated.


## Video Guide

[Watch the Video](https://drive.google.com/file/d/14uEygiFRnf24SA8sSyoOl4IOcJMX3Sf7/view?usp=sharing)


