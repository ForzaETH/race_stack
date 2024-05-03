# Docker Structure and Guidelines

## Structure
First, a base docker image is defined through the [base Dockerfile](./Dockerfile.base_x86). This can be extended then for development with the simulator, with the [sim dockerfile](./Dockerfile.sim_x86) or for development on the physical car with the [nuc dockerfile](./Dockerfile.nuc). 
While the previously linked docker files are meant for x86 platforms, there are parallel dockerfiles defined for arm platforms ([base](./Dockerfile.base_arm), [sim](./Dockerfile.sim_arm), [car](./Dockerfile.jet)).

They are meant to be built through the [docker-compose.yaml](../docker-compose.yaml), and they are either meant to be used in a VS Code devcontainer ([devcontainer.json](../.devcontainer/devcontainer.json) here) or with the accompanying scripts ([main](./main_dock.sh), [secondary](./sec_dock.sh), [main attach](./main_attach_dock.sh)).

**NOTE**: it is suggested to set a static IP for the robot with the ROS_HOSTNAME environment variable, so that the IP of the robot is always the same, as from [networking structure](../stack_master/checklists/networking.md).
Setting up a static ip at this moment will allow you to set extra computers (that can ping such IP) to listen to the ROS messags on the robot, enabling fast and quick development. 


## How to use (simulator)
**Note**: this following tutorial assumes you are using an x86 platform (e.g. Intel CPU). In case you have an ARM platform (e.g. NVIDIA Jetson) use the dockerfiles substituting `_x86` with `_arm`.

**Note 2**: docker is assumed to be installed and runnable without sudo (e.g. on Linux see [Post-Installation steps](https://docs.docker.com/engine/install/linux-postinstall/)).

**Step 1/4**: First build the base docker image with `docker compose`:
```bash
docker compose build base_x86
```
For an x86 build on a 8th-gen i7 CPU, this took roughly 45 minutes (though it depends also on connection speed). So sit back and have a coffee! ☕ 

**Step 2/4**: Then export the needed environment variables and build the simulator container:
```bash
export UID=$(id -u)
export GID=$(id -g)
docker compose build sim_x86
```
This should take much less time, less than 1 minute.

**Step 3/4**: Create  folder structure that resembles the following. Note that it is two folders up from the position of the `race_stack`.

```bash
<race_stack directory>/../
...
├── cache
│   └── noetic
│       ├── build
│       ├── devel
│       └── logs
└ ...
```

It can be done with the following command:
```bash
cd <race_stack folder>
mkdir -p ../cache/noetic/build ../cache/noetic/devel ../cache/noetic/logs
```

**Note**: In case it is particularly problematic to create such a folder structure two parent directories up, it can also be created just one directoy up, but the cache mounting point must be changed in the devcontainer.json for the build, devel and log folder.  

**Step 4/4**: Launch the VS Code devcontainer. This can be done by opening the race_stack folder in VS Code, launching the command palette with the shortcut `Ctrl`+`Shift`+`p`, and selecting `Dev Containers: Rebuild and Reopen in Container`. Make sure that the field `image` has the correct name of the simulator docker image that you want to use, name that can be found in the [`docker-compose.yaml`](../docker-compose.yaml) in the field `image`. In this case, it is `race_stack_sim_x86` .

## How to use (car)
**Note**: this following tutorial assumes you are using an x86 platform (e.g. Intel CPU). In case you have an ARM platform (e.g. NVIDIA Jetson) use the dockerfiles substituting `_x86` with `_arm` and change `_nuc` to `_jet`.


**Note 2**: docker is assumed to be installed and runnable without sudo (e.g. on Linux see [Post-Installation steps](https://docs.docker.com/engine/install/linux-postinstall/)).

**Step 1/4**: First build the base docker image with `docker compose`:
```bash
docker compose build base_x86
```

**Step 2/4**: Then export the needed environment variables and build the car container:
```bash
export UID=$(id -u)
export GID=$(id -g)
docker compose build nuc
```

**Step 3/4**: Create  folder structure that resembles the following. Note that it is two folders up from the position of the `race_stack`.

```bash
<race_stack directory>/../
...
├── cache
│   └── noetic
│       ├── build
│       ├── devel
│       └── logs
└ ...
```

It can be done with the following command:
```bash
cd <race_stack folder>
mkdir -p ../cache/noetic/build ../cache/noetic/devel ../cache/noetic/logs
```

**Note**: In case it is particularly problematic to create such a folder structure two parent directories up, it can also be created just one directoy up, but the cache mounting point must be changed in the devcontainer.json for the build, devel and log folder.  

**Step 4/4**: 
Since we prefer to not launch the car nodes with a VS Code session attached to them, we will in this case use the scripts present in the folder [.docker_utils](../.docker_utils). First make sure that the environment variable `IMAGE` in the [`main_dock.sh`](./main_dock.sh) has the correct name of the car docker image that you want to use, name that can be found in the [`docker-compose.yaml`](../docker-compose.yaml) in the field `image`. In this case, it is `race_stack_nuc`.
Then, for the first time launching the setup, use the [`main_dock.sh`](./main_dock.sh) file:
```bash
cd <race_stack folder>
export RACE_STACK_ROOT=$(pwd)
source ./.devcontainer/.install_utils/xauth_setup.sh
./.docker_utils/main_dock.sh
```

Then, **in the new terminal inside the container**, run the post installation script, that will allso build your system:
```bash
cd <race_stack folder>
./.devcontainer/.install_utils/post_installation.sh
```
You are now good to go, you can also exit the container (with `Ctrl`+`D` or exit). 

To restart and attach to the container, use the attaching script [`main_attach_dock.sh`](./main_attach_dock.sh) and to obtain multiple terminals in this same container you can use  [`sec_dock.sh`](./sec_dock.sh) (or terminal multiplexers).
```bash
cd <race_stack folder>
# in case main dock is not on 
./.docker_utils/main_attach_dock.sh

# in case you want additional terminals
./.docker_utils/sec_dock.sh
```

## How to use GUI applications with the container
To have more information on how to use GUI applications with the container, please refer to the [GUI applications documentation](./README_GUI.md).


---
[Go back to the main README](../README.md)
