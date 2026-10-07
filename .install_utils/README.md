# Docker Structure and Guidelines
**Note:** Currently only installation via docker has been tested and is supported.

## Clone Repo
Be aware to clone the ROS 2 branch!
```bash
git clone -b ros2-humble --recurse-submodules git@github.com:ForzaETH/race_stack.git 
cd race_stack
```

## Structure
The docker image is defined with the [Dockerfile](../.devcontainer/Dockerfile).


**NOTE**: it is suggested to set a static IP for the robot with the ROS_HOSTNAME environment variable, so that the IP of the robot is always the same, as from [networking structure](../stack_master/checklists/networking.md).
Setting up a static ip at this moment will allow you to set extra computers (that can ping such IP) to listen to the ROS messags on the robot, enabling fast and quick development. 


## Build
**Note**: docker is assumed to be installed and runnable without sudo (e.g. on Linux see [Post-Installation steps](https://docs.docker.com/engine/install/linux-postinstall/)).
The build process is managed by the Makefile. The command `default_setup` creates a cache folder (`setup_cache`) and a .env file (`export_env`) before it builds the container (`build`). It also detects if you are on a x86 or arm machine and builds the correct image. 

```bash
make default_setup
```

## How to use (VSCode devcontainer)

**Open the devcontainer**
**Note** this step must be done strictly after the completion of build step, as otherwise the permission file mounted in the devcontainer might be wrong.
Open the devcontainer on the car, first by opening up VSCode, then connecting to the car with the remote connection button to the bottom left (Connect to Host...), then open the race stack folder, and reopen in the devcontainer. On your laptop you can directly click reopen in devcontainer.


Once in the devcontainer, open a terminal and export the `DISPLAY` variable number. For example:
```bash
export DISPLAY=localhost:10.0
```

You can now enjoy a terminal with GUI forwarding! If you need multiple GUI applications, make sure to export the `DISPLAY` variable in each terminal you want to use GUI applications in.

## How to use (docker container)
**Set up X forwarding and launch the container**
To get GUI application properly forwarded, run the additional setup script that sets up the xauth file for the container. The launch command does this automatically. If you are on an arm machine (such as a macbook) this step is omitted. It is recommended to use [this](https://github.com/ETH-PBL/remote-novnc) for display support. To launch the container you can then run:

```bash
cd <race_stack folder>
make launch
```
This spins up the container and runs some post installation steps. This is only done the first time and the next time the script will detect the correct container and restarts it. After the post installation finished you can run `make launch` again to attach a terminal to the container.



## How to use GUI applications with the container
To have more information on how to use GUI applications with remote containers, please refer to the [GUI applications documentation](./README_GUI.md).

---
[Go back to the main README](../README.md)

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
