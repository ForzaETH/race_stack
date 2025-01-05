## Description

Interface (driver) software, including ROS node, for inertial sensors from [MicroStrain by HBK](https://microstrain.com), developed in Williston, VT.

Implemented using the MicroStrain Inertial Protocol SDK ([`mip_sdk`](https://github.com/LORD-MicroStrain/mip_sdk))

## Table of Contents

* [ROS Wiki](#ros-wiki)
* [ROS vs ROS2 versions](#ros-vs-ros2-versions)
* [Packages](#packages)
* [Install Instructions](#install-instructions)
    * [Buildfarm](#buildfarm)
    * [Source](#source)
    * [Udev Rules](#udev-rules)
* [Building From Source](#building-from-source)
* [Run Instructions](#run-instructions)
    * [Single Device](#launch-the-node-and-publish-data)
    * [Multiple Devices](#publish-data-from-two-devices-simultaneously)
* [Docker Development](#docker-development)
    * [VSCode](#vscode)
    * [Make](#make)
* [Shared Codebases](#shared-codebases)
* [Previous Versions](#previous-versions)
* [Licenses](#license)

## ROS Wiki

For more information on the data published and services available see our [ROS wiki](https://wiki.ros.org/microstrain_inertial_driver) page

## ROS vs ROS2 Versions

Note that this branch contains the ROS2 implementation for the packages. If you are looking for the ROS version, you should go to the [`ros`](https://github.com/LORD-MicroStrain/ROS-MSCL/tree/ros) branch

## Packages

This repo contains the following packages:

* [`microstrain_inertial_driver`](./microstrain_inertial_driver) -- ROS node that will communicate with the devices
* [`microstrain_inertial_msgs`](./microstrain_inertial_msgs) -- Collection of messages produced by the `microstrain_inertial_driver` node
* [`microstrain_inretial_examples`](./microstrain_inertial_examples) -- Collection of examples that show how to interact with the `microstrain_inertial_driver` node. Currently contains one simple C++ and python subscriber node
* [`microstrain_inertial_rqt`](./microstrain_inertial_rqt) -- Collection of RQT plugins to view the status of inertial devices when running the `microstrain_inertial_driver`

## Install Instructions

### Buildfarm

As of `v2.0.5` this package is being built and distributed by the ROS build farm. If you do not need to modify the source, it is recommended to install directly from the buildfarm by running the following commands where `ROS_DISTRO` is the version of ROS you are using such as `galactic` or `humble`:

Driver:
```bash
sudo apt-get update && sudo apt-get install ros-ROS_DISTRO-microstrain-inertial-driver
```

RQT:
```bash
sudo apt-get update && sudo apt-get install ros-ROS_DISTRO-microstrain-inertial-rqt
```

For more information on the ROS distros and platforms we support, please see [index.ros.org](https://index.ros.org/r/microstrain_inertial/github-LORD-MicroStrain-microstrain_inertial/#humble)


### Source

If you need to modify the source of this repository, or are running on a platform that we do not support, you can build from source by following the [Building From Source](#building-from-source) guide below.


#### **IMPORTANT NOTE ABOUT CLONING**

This repo takes advantage of git submodules in order to share code between ROS versions. When cloning the repo, you should clone with the `--recursive` flag to get all of the submodules.

If you have already cloned the repo, you can checkout the submodules by running `git submodule update --init --recursive` from the project directory

The [CMakeLists.txt](./microstrain_inertial_msgs/CMakeLists.txt) will automatically checkout the submodule if it does not exist, but it will not keep it up to date. In order to keep up to date, every
time you pull changes you should pull with the `--recurse-submodules` flag, or alternatively run `git submodule update --recursive` after you have pulled changes


## Building from source

1. Install ROS2 and create a workspace: [Configuring Your ROS2 Environment](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html)

2. Clone the repository into your workspace:
    ```bash
    git clone --recursive --branch ros2 https://github.com/LORD-MicroStrain/microstrain_inertial.git ~/your_workspace/src/microstrain_inertial
    ```

3. Install rosdeps for this package: `rosdep install --from-paths ~/your_workspace/src -i -r -y`

4. Build your workspace:

    ```bash        
    cd ~/your_workspace
    colcon build
    source ~/your_workspace/install/setup.bash
    ```
   The source command will need to be run in each terminal prior to launching a ROS node.

## Udev Rules

**NOTE**: If installing from the buildfarm, the udev rules will be installed automatically

This driver comes with [udev rules](https://wiki.debian.org/udev) that will create a symlink for all microstrain devices.
To install the rules. Download the [udev](./microstrain_inertial_driver/debian/udev) file from this repo and copy it to
`/etc/udev/rules.d/100-microstrain.rules`

Once the udev rules are installed, the devices will appear as follows in the file system, where {serial} is the serial number of the device:

* `/dev/microstrain_main` - Most recent non-GQ7 device, or the main port of a GQ7 connected. **NOTE**: Do not use this rule with multiple devices as it gets overridden with multiple devices.
* `/dev/microstrain_aux` - Most recent GQ7 aux port connected. **NOTE**: Do not use this rule with multiple devices as it gets overridden with multiple devices.
* `/dev/microstrain_main_{serial}` - All non-GQ7 devices, and the main port of GQ7 devices
* `/dev/microstrain_aux_{serial}` - The aux port of GQ7 devices

## Run Instructions

#### Launch the node and publish data
The following command will launch the driver. Keep in mind each instance needs to be run in a separate terminal.
```bash
ros2 launch microstrain_inertial_driver microstrain_launch.py
```

The node has some optional launch parameters that can be specified from the command line in the format `param:=value`
- `namespace` : namespace that the driver will run in. All services and publishers will be prepended with this, default: `/`
- `node_name` : name of the driver, default: `microstrain_inertial_driver`
- `debug`     : output debug logs, default: `false`
- `params_file` : path to a parameter file to override the default parameters stored in [`params.yml`](./microstrain_inertial_driver/microstrain_inertial_driver_common/config/params.yml), default: [`empty.yml`](./microstrain_inertial_driver/config/empty.yml)
    
#### Publish data from two devices simultaneously  

1. Create the following files somewhere on your system (we will assume they are stored in the `~` directory):
    1. `~/sensor_a_params.yml` with the contents:
        ```yaml
        port: /dev/ttyACM0
        ```
    2. `~/sensor_b_params.yml` with the contents:
        ```yaml
        port: /dev/ttyACM1
        ```
2. In two different terminals:
    ```bash    
    ros2 launch microstrain_inertial_driver microstrain_launch.py node_name:=sensor_a_node namespace:=sensor_a params_file:="~/sensor_a_params.yml"
    ```
    ```bash    
    ros2 launch microstrain_inertial_driver microstrain_launch.py node_name:=sensor_b_node namespace:=sensor_b params_file:="~/sensor_b_params.yml"
    ```

This will launch two nodes that publish data to different namespaces:
- `/sensor_a`, connected over port: `/dev/ttyACM0`
- `/sensor_b`, connected over port: `/dev/ttyACM1`

An example subscriber node can be found in the [MicroStrain Examples](./microstrain_inertial_examples) package.

#### Lifecycle Node

This package also provides a lifecycle node implementation. This version of the driver can be launched by running:
```bash
ros2 launch microstrain_inertial_driver microstrain_lifecycle_launch.py
```

This launch file accepts all of the same arguments as the above node as well as:
- `configure` : If set to the exact string `true` the driver will automatically transition into the configure state.
- `activate`  : If set to the exact string `true` the driver will automatically transition into the activate state.

Additionally, the node may be transitioned anytime after startup using the following commands (note that the `namespace` and `name` parameters will affect the node name in the following commands):

- Transition to configure state: 
    ```bash
    ros2 lifecycle set /microstrain_inertial_driver_node configure
    ```

- Transition to active state: 

    ```bash
    ros2 lifecycle set /microstrain_inertial_driver_node activate
    ```

You can stop data from streaming by putting the device into the "deactivate" state.  Both the "cleanup" and "shutdown" states will disconnect from the device and close the raw data log file (if enabled.)

## Docker Development

### VSCode

The easiest way to develop in docker while still using an IDE is to use VSCode. Follow the steps below to develop on this repo in a docker container.

1. Install the following dependencies:
    1. [VSCode](https://code.visualstudio.com/)
    1. [Docker](https://docs.docker.com/get-docker/)
1. Open VSCode and install the following [plugins](https://code.visualstudio.com/docs/editor/extension-marketplace):
    1. [VSCode Remote Containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
1. Open this directory in a container:
    1. Open the `microstrain_inertial` directory in VSCode
    1. Click the green `><` icon in the bottom left corner of the window
    1. Choose `Reopen In Container`
1. Once the project is open in the container, it will take some time to automatically install the rosdeps inside the container, you can then build and run the container by following step 4 of [Building from source](#building-from-source)

### Make

If you are comfortable working from the command line, or want to produce your own runtime images, the [Makefile](./devcontainer/Makefile) in the [.devcontainer](./devcontainer) 
directory can be used to build docker images, run a shell inside the docker images and produce a runtime image. Follow the steps below to setup your environment to use the `Makefile`

1. Install the following dependencies:
    1. [Make](https://www.gnu.org/software/make/)
    1. [Docker](https://docs.docker.com/get-docker/)
    1. [qemu-user-static](https://packages.ubuntu.com/bionic/qemu-user-static) (**only for multiarch builds**)
        1. Run the following command to register the qemu binaries with docker: `docker run --rm --privileged multiarch/qemu-user-static:register`

The `Makefile` exposes the following tasks. They can all be run from the `.devcontainer` directory:
* `make build-shell` - Builds the development docker image and starts a shell session in the image allowing the user to develop and build the ROS project using common commands such as `catkin_make`
* `make image` - Builds the runtime image that contains only the required dependencies and the ROS node.
* `make clean` - Cleans up after the above two tasks

## Shared codebases

Both the `ros` and `ros2` branches share most of their code by using git submodules. The following submodules contain most of the actual implementations:

* [microstrain_inertial_driver_common](https://github.com/LORD-MicroStrain/microstrain_inertial_driver_common/tree/main) submoduled in this repo at `microstrain_inertial_driver/microstrain_inertial_driver_common`
* [microstrain_inertial_msgs_common](https://github.com/LORD-MicroStrain/microstrain_inertial_msgs_common/tree/main) submoduled in this repo at `microstrain_inertial_msgs/microstrain_inertial_msgs_common`
* [microstrain_inertial_rqt_common](https://github.com/LORD-MicroStrain/microstrain_inertial_rqt_common/tree/main) submoduled in this repo at `microstrain_inertial_rqt/microstrain_inertial_rqt_common`

## Previous Versions

Previous versions of the driver were released as [tags](https://github.com/LORD-MicroStrain/microstrain_inertial/tags) on Github. They can also be found in specific branches:

* [`ros2-3.x.x`](https://github.com/LORD-MicroStrain/microstrain_inertial/tree/ros2-3.x.x) contains the most recent code before the standardizing refactor
* [`ros2-2.x.x`](https://github.com/LORD-MicroStrain/microstrain_inertial/tree/ros2-2.x.x) contains the most recent code before the MIP SDK refactor

## License

Different packages in this repo are released under different licenses. For more information, see the LICENSE files in each of the package directories.

Here is a quick overview of the licenses used in each package:

| Package                                                                  | License |
| ------------------------------------------------------------------------ | ------- |
| [microstrain_inertial_driver](./microstrain_inertial_driver/LICENSE)     | MIT     |
| [microstrain_inertial_msgs](./microstrain_inertial_msgs/LICENSE)         | MIT     |
| [microstrain_inertial_rqt](./microstrain_inertial_rqt/LICENSE)           | BSD     |
| [microstrain_inertial_examples](./microstrain_inertial_examples/LICENSE) | MIT     |
