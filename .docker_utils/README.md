# Docker Structure and Guidelines
**Note:** Currently only installation via docker has been tested and is supported.

## Clone Repo
Be aware to clone the ROS 2 branch!
```bash
git clone -b ros2-humble --recurse-submodules https://github.com/Ingenuity-Labs-Racing/ForzaETH_Race_Stack.git 
```

## Structure
The base docker image is defined with the [Dockerfile](../.devcontainer/Dockerfile). Two version exist for different architectures. 
- [Dockerfile.x86](../.devcontainer/Dockerfile.x86) for x86 machines, such as most Intel machines
- [Dockerfile.arm](../.devcontainer/Dockerfile.x86) for ARM machines, such as the Nvidia Jetson


**NOTE**: it is suggested to set a static IP for the robot with the ROS_HOSTNAME environment variable, so that the IP of the robot is always the same, as from [networking structure](../stack_master/checklists/networking.md).
Setting up a static ip at this moment will allow you to set extra computers (that can ping such IP) to listen to the ROS messags on the robot, enabling fast and quick development. 


## How to use (docker container)
**Note**: this following tutorial assumes you are using an x86 platform (e.g. Intel CPU). TODO: arm instructions

**Note 2**: docker is assumed to be installed and runnable without sudo (e.g. on Linux see [Post-Installation steps](https://docs.docker.com/engine/install/linux-postinstall/)).

**Step 1/5: Build the container**
First export the following environment variables
```bash
export UID=$(id -u)
export GID=$(id -g)
```
then, in the same terminal, build the docker image with `docker compose`:
**For x86 platforms**:
```bash
docker compose build nuc
```
**For arm platforms**:
```bash
docker compose build jet
```

**Step 2/5: create the folder structure for caching colcon builds**
Create a folder structure that resembles the following. Note that it is a folder up from the position of the `race_stack`.

```bash
<race_stack directory>/../
...
├── cache
│   └── humble
│       ├── build
│       ├── install
│       └── log
└ ...
```

It can be done with the following command:
```bash
cd <race_stack folder>
mkdir -p ../cache/humble/build ../cache/humble/install ../cache/humble/log
```

**Step 3/5: Set up the launch script**
Check that the `FORZAETH_DIR` variable is correctly set inside of [`main_dock.sh`](./main_dock.sh) to the path of the `race_stack` folder. This is needed for the simulator to work correctly. You can check the full path with the following command
```bash
cd <race_stack folder>
pwd
```
Then copy the output of the `pwd` command and paste it in the `FORZAETH_DIR` variable in the [`main_dock.sh`](./main_dock.sh) file.

Furthermore, change the name of the image accordingly depending on if you are using an x86 or an arm platform. 



**Step 4/5: Set up X forwarding and launch the container**
To get GUI application properly forwarded, run the additional setup script that sets up the xauth file for the container. This can be done by running the following command:
```bash
cd <race_stack folder>
source .devcontainer/xauth_setup.sh
```

> **Note:** `main_dock.sh` now performs this xauth setup itself, so for this plain-docker flow the `source .devcontainer/xauth_setup.sh` step above is optional. It is still required for the VSCode devcontainer flow below.

Then, in the same terminal, launch the docker container with the following command
```bash
./.docker_utils/main_dock.sh
```

You will now have access to a terminal inside the container. 
Here, run the postcreate command with the following line
```bash
cd ~/ws/src/race_stack
./.install_utils/post_create_command.sh
```
This will setup the final packages and configurations needed for the container to work correctly.

**Step 5/5: Open additional terminals, reopen a closed terminal**
You can now attach multiple terminals to the container with the secondary scritp:
```bash
# in a terminal outside of the container
cd <race_stack folder>
./.docker_utils/sec_dock.sh
```

Once the main docker container is closed, you can also reopen the same one with the attach script:
```bash
# in a terminal outside of the container
cd <race_stack folder>
./.docker_utils/main_attach_dock.sh
```


## How to use (VSCode devcontainer)

**Note**: this following tutorial assumes you are using an x86 platform (e.g. Intel CPU). TODO: arm instructions

**Note 2**: docker is assumed to be installed and runnable without sudo (e.g. on Linux see [Post-Installation steps](https://docs.docker.com/engine/install/linux-postinstall/)).

**Step 1/5: setup the X forwarding** 

In case you want to use the VSCode devcontainer in a remote machine, and you want graphical application to be forwarded, you need to setup the xauth file for the container. This is not necessary if only using the container locally, for example to run the simulator on the local machine. 

This can be done by running the following command:

```bash
cd <race_stack folder>
source .devcontainer/xauth_setup.sh
```

Make sure to do this in a remote terminal with X forwarding enabled, as described in the [GUI applications documentation](./README_GUI.md).

Then print out the `DISPLAY` variable number with the following command, in order to remember it for later use:
```bash
echo $DISPLAY
```
an example output could be 
```
localhost:10.0
```
**Note**: this container must then be left running.

**Step 2/5: create the folder structure for caching colcon builds**

Create a folder structure that resembles the following. This is done so that `colcon build` artifacts are mounted onto the machine hosting the Docker container. Note that it is a folder up from the position of the `race_stack`.

```bash
<race_stack directory>/../
...
├── cache
│   └── humble
│       ├── build
│       ├── install
│       └── log
└ ...
```

It can be done with the following command:
```bash
cd <race_stack folder>
mkdir -p ../cache/humble/build ../cache/humble/install ../cache/humble/log
```

**Step 3/5: Build the container**

Export environment variables before building the Docker container:
```bash
export UID=$(id -u)
export GID=$(id -g)
```

In a terminal connected to the remote machine you want to use, move to the location of the racestack, and build the docker container with the compose command:
**For x86 platforms**:
```bash
docker compose build nuc
```
**For arm platforms**:
```bash
docker compose build jet
```

Make sure the `image` attribute in the [devcontainer.json](../.devcontainer/devcontainer.json) matches the architecture you require

- `"image": "jet_forzaeth_racestack_ros2:humble",` for ARM 
- `"image": "nuc_forzaeth_racestack_ros2:humble",` for x86

For example, for x86, it should be:
```json5
//<race_stack_directory>/.devcontainer/devcontainer.json
...
    "image": "nuc_forzaeth_racestack_ros2", 
...
```

**Step 4/5: Open the devcontainer**

**Note** this step must be done strictly after the completion of step 1, as otherwise the permission file mounted in the devcontainer might be wrong.

1. **For use with display forwarding on a remote machine**:

    Open the devcontainer on the car, first by opening up VSCode, then connecting to the car with the remote connection button to the bottom left (Connect to Host...), then open the race stack folder, and reopen in the devcontainer.
    Once in the devcontainer, open a terminal and export the `DISPLAY` variable number. For example:
    ```bash
    export DISPLAY=localhost:10.0
    ```
    Note: the full <name>:<port> couple is needed as from step 1.

    You can now enjoy a terminal with GUI forwarding! If you need multiple GUI applications, make sure to export the `DISPLAY` variable in each terminal you want to use GUI applications in.

2. **For local use**:

    Open the race stack folder in VSCode. Hit Ctrl+Shift+P to open the command palette, and select **Dev Containers: Reopen in Container**

**Step 5/5: Open additional terminals** 
You can also attach multiple terminals to the container with the secondary script, from outside VSCode:
```bash
# in a terminal outside of the container
cd <race_stack folder>
./.docker_utils/sec_dock.sh
```
The name used inside the `sec_dock.sh` file must be the same as the one set in the `image` field of the `devcontainer.json` in step 3.

## How to use GUI applications with the container
To have more information on how to use GUI applications with remote containers, please refer to the [GUI applications documentation](./README_GUI.md).

---
[Go back to the main README](../README.md)

**Bonus: Verify your installation**

To verify the installation was successful, make sure you are navigate in the `~/ws` directory inside the Docker container (whether in VSCode Dev Container or plain Docker shell) and do `colcon build` to ensure all packages can be built.

Next, test the time trial by launching the base system followed by the time trial: 

```bash
ros2 launch stack_master base_system_launch.xml racecar_version:=SIM map_name:=glc_ot_ez sim:=True

ros2 launch stack_master time_trials_launch.xml racecar_version:=SIM
```

You should see the car driving around the track!

### To manually install dependencies (should not be necessary if you build in docker):
```bash
# general ubuntu packages dependencies
xargs sudo apt-get install -y < ~/ws/src/race_stack/.install_utils/linux_req/linux_req.txt

# ubuntu packages dependencies for sim
xargs sudo apt-get install -y < ~/ws/src/race_stack/.install_utils/linux_req/linux_req_sim.txt

# ubuntu packages dependencies for car
xargs sudo apt-get install -y < ~/ws/src/race_stack/.install_utils/linux_req/linux_req_car.txt

# python dependencies
pip install -r ~/ws/src/race_stack/.install_utils/python_req.txt

# setup f1tenth_gym
source ~/ws/src/race_stack/.install_utils/f110_sim_setup.sh
```
