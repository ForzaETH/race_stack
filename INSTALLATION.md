# ForzaETH Race Stack Installation
After following the [prerequisites](#prerequisites) you will need to choose either [Car installation](#car-installation) or [Sim installation](#sim-installation). We recommend the use of [Docker](#docker) for either installation.

## Prerequisites 
To run the ForzaETH race stack make sure that you have assembled and setup a car following the [official F1TENTH instructions](https://f1tenth.org/build). Follow the instructions up to the point *Install F1TENTH Driver Stack » 3. F1TENTH Driver Stack Setup » 1. udev Rules Setup*.

For both the car and the sim installation, be sure to have installed:
  - [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
  - [Catkin Build Tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

**Note**: 
The two installations are not needed in the case of the [Docker](#docker) setup.

**Note**:
Be sure to have included the sourcing lines in your `~/.bashrc` file, in order to properly setup ROS in every terminal you open. 
The two lines to be added are (if you are using `bash`)
```
source /opt/ros/noetic/setup.bash
source <path to your catkin_ws>/devel/setup.bash
```


## Docker
For quick deployment, a docker image can be used.
The full docker structure and guidelines on how to use both in simulation and in the real platform can be found in the [Docker Guidelines README](./.docker_utils/README.md).

For a quick example of the race stack in action, first build the base docker image with `docker compose`:
```bash
docker compose build base_x86
```

Then export the needed environment variables and build the simulator container:
```bash
export UID=$(id -u)
export GID=$(id -g)
docker compose build sim_x86
```

Then check that the following folder structure is existing:
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
It can be created from the command line, for example:
```bash
cd <race_stack folder>
mkdir -p ../cache/noetic/build ../cache/noetic/devel ../cache/noetic/logs
```
Then launch the devcontainer from VS Code

To now test the simulator, launch the simulator with base system with the following command:
```bash
roslaunch stack_master base_system.launch sim:=true map_name:=test_map
```

and then, in a new terminal, launch the timetrials system with the following command:
```bash
roslaunch stack_master time_trials.launch racecar_version:=NUC2
```
For more information on how to run the different modules on the car, refer to the [`stack_master`](./stack_master/README.md) README or to the READMEs in the [checklist](./stack_master/checklists/) directory.

If you want a native source installation follow the installation steps below.

## Native installation
The following steps assume you have a catkin workspace set up and are working from within the ```src``` folder. For example:
```
cd ~/catkin_ws/src
```
### Car installation
#### [Step 1 of 5] 
First you'll have to clone the repository with all the submodules in it
```
git clone --recurse-submodules git@github.com:ForzaETH/race_stack.git 
cd race_stack
```
From now on the installation assumes you are in the `race_stack` folder.

#### [Step 2 of 5]
Install all the dependencies
```
# ubuntu packages dependencies
xargs sudo apt-get install -y < ./.install_utils/linux_req_car.txt

# python dependencies
pip install -r ./.devcontainer/.install_utils/requirements.txt
```


#### [Step 3 of 5]
Then you will need to install the cartographer version which we modified (and which was just downloaded as submodule)
```
chmod +x ./.devcontainer/.install_utils/cartographer_setup.sh
sudo ./.devcontainer/.install_utils/cartographer_setup.sh
```
#### [Step 4 of 5]
Similarly you will need to install the SynPF particle filter (which was just downloaded as submodule)
```
chmod +x ./.devcontainer/.install_utils/pf_setup.sh
sudo ./.devcontainer/.install_utils/pf_setup.sh
```
####  [Step 5 of 5]

**Optional**: if you want you can erase the simulator folder, so to not install it on the car too. 
It can be done with the following command 
```
# Optional
rm -rf ./base_system/f110-simulator
``` 

Then build the whole system!
```
catkin build
```

The car is now ready to be tested. For examples on how to run the different modules on the car, refer to the [`stack_master` README](./stack_master/README.md). As a further example, the [time-trials](./stack_master/checklists/TimeTrials.md) or the [head-to-head](./stack_master/checklists/HeadToHead.md) checklists are a good starting point.

### Sim installation
#### [Step 1 of 2]
First you'll have to clone the repository
```
git clone --recurse-submodules git@github.com:ForzaETH/race_stack.git 
cd race_stack
```
From now on the installation assumes you are in the `race_stack` folder.

#### [Step 2 of 3]
Install all the dependencies
```
# ubuntu packages dependencies
xargs sudo apt-get install -y < ./.devcontainer/.install_utils/linux_req_sim.txt

# python dependencies
pip install -r ./.devcontainer/.install_utils/requirements.txt
```
#### [Step 3 of 3]
**Optional**: if you want you can erase the simulator folder, so to not install it on the car too. 
It can be done with the following command 
```
# Optional
rm -rf ./base_system/f1tenth_system
``` 

Then build the whole system!
```
catkin build
```

After installation you can test simulation functionalities, for example running the [time-trials](./stack_master/checklists/TimeTrials.md) or the [head-to-head](./stack_master/checklists/HeadToHead.md) checklists. For a more general overview of how to use the `race_stack` refer to the [`stack_master` README](./stack_master/README.md).
