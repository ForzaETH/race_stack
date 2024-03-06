# New F1TENTH Race Stack at Center for Project Based Learning

## Installation instructions 
After following the [prerequisites](#prerequisites) you will need to choose either [Car installation](#car-installation) or [Sim installation](#sim-installation).

### Prerequisites 
For both the car and the sim installation, be sure to have installed:
  - [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
  - [Catkin Build Tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

**Note**:
Be sure to have included the sourcing lines in your `~/.bashrc` file, in order to properly setup ROS in every terminal you open. 
The two lines to be added are (if you are using `bash`)
```
source /opt/ros/noetic/setup.bash
source <path to your catkin_ws>/devel/setup.bash
```

### Car installation
#### [Step 1 of 5] 
First you'll have to clone the repository with all the submodules in it
```
cd ~/catkin_ws/src
git clone --recurse-submodules git@git.ee.ethz.ch:pbl/research/f1tenth/race_stack.git 
cd race_stack
```

#### [Step 2 of 5]
Install all the dependencies
```
# ubuntu packages dependencies
xargs sudo apt-get install -y < ~/catkin_ws/src/race_stack/.devcontainer/.install_utils/linux_req_car.txt

# python dependencies
pip install -r ~/catkin_ws/src/race_stack/.devcontainer/.install_utils/requirements.txt
pip install ~/catkin_ws/src/race_stack/f110_utils/libs/ccma  
```


#### [Step 3 of 5]
Then you will need to install the cartographer version which we modified (and which was just downloaded as subrepo)
```
chmod +x .devcontainer/.install_utils/cartographer_setup.sh
cd ~/catkin_ws
sudo src/race_stack/.devcontainer/.install_utils/cartographer_setup.sh
```
#### [Step 4 of 5]
Similarly you will need to install the MIT particle filter version which we modified (and which was just downloaded as subrepo)
```
chmod +x ~/catkin_ws/src/race_stack/.devcontainer/.install_utils/pf_setup.sh
cd ~/catkin_ws
sudo src/race_stack/.devcontainer/.install_utils/pf_setup.sh
```
####  [Step 5 of 5]

**Optional**: if you want you can erase the simulator folder, so to not install it on the car too. 
It can be done with the following command 
```
# Optional
rm -rf ~/catkin_ws/src/race_stack/base_system/f110-simulator
``` 

Then build the whole system!
```
catkin build
```

### Sim installation
#### [Step 1 of 2]
First you'll have to clone the repository
```
cd ~/catkin_ws/src
git clone git@gitlab.ethz.ch:pbl/research/f1tenth_2/race_stack.git 
```
#### [Step 2 of 3]
Install all the dependencies
```
# ubuntu packages dependencies
xargs sudo apt-get install -y < ~/catkin_ws/src/race_stack/.devcontainer/.install_utils/linux_req_sim.txt

# python dependencies
pip install -r ~/catkin_ws/src/race_stack/.devcontainer/.install_utils/requirements.txt
pip install <path_to_race_stack>/f110_utils/libs/ccma
```
#### [Step 3 of 3]
**Optional**: if you want you can erase the simulator folder, so to not install it on the car too. 
It can be done with the following command 
```
# Optional
rm -rf ~/catkin_ws/src/race_stack/base_system/pbl_f110_system
``` 

Then build the whole system!
```
catkin build
```

## Docker
For quick deployment, a docker image could also be used. It is automatically built and tested for every commit on the `develop` branch.
The full docker structure and guidelines on how to use can be found in the [Docker Guidelines README](./.docker_utils/README.md).

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
<race_stack directory>/../../
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
mkdir -p ../../cache/noetic/build ../../cache/noetic/devel ../../cache/noetic/logs
```
Then launch the devcontainer from VS Code

If you want a native source installation the follow the Car installation steps above.
