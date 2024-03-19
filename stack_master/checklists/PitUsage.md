## Pit Usage Guide

**Note**: Throughout this README, NUCX refers to a generic identificative name for a physical car.

Once you are connected to the Zerotier SDWAN as in [here](./networking.md) (or once you have fixed IPs assigned to the cars) you are able to SSH into the car. But you will want to interact/visualize/debug the car from your laptop aka _the pit_. 

Therefore you will need the `pit_starter.sh` from [the utils directory](../../f110_utils/scripts/pit_starter/pit_starter.sh). To use it:
- `cd <race_stack folder>/f110_utils/scripts/pit_starter`
- `source pit_starter.sh <YOUR_ZEROTIER/FIXED_IP> <NUCX> rviz`
- Enjoy freshly sourced NUCX and runs the rviz with the correct config.
- Pro tip: If you only want to source the NUCX then dont use the `rviz` argument

For convenience, it's nice to set up some aliases in your bashrc (Bash shell config file) to speed-up different things. Your bashrc is found in `~/.bashrc`. An example is below. 

**NOTE**: the path of your race_stack is assumed to be in `~/catkin_ws/src`, but it might not be the case for you. So change if needed.

---

First, get your Zerotier IP handy, if applicable. You can do so by running `ifconfig` in the terminal and see the `inet` address for the network adapter that starts with `zt`.

```bash
#Source setup bashes
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# aliases for different ROS distros.
alias sros1="source /opt/ros/noetic/setup.bash"
alias sf1="sros1; source ~/catkin_ws/devel/setup.bash"  # for convenience, no need to source ROS1 first.
alias sauce="sf1"

# aliases to get into cars faster
alias sshnuc2="ssh -XC <car_username@<car_fixed_IP>"

# TODO : Check your Zerotier IP first! if it starts with 192.168.192 then you can use the following line, otherwise you need to change it.
export ZEROTIER_IP=$( ifconfig | awk '/inet 192.168.192/ {print $2}' )

# Helpers for Pit Starter Source (if you just want to set the environment variables in your system)
alias pit1='source ~/catkin_ws/src/race_stack/f110_utils/scripts/pit_starter/pit_starter.sh "$ZEROTIER_IP" NUC2'

# If you want to source and open RViz too
alias pit_rviz2='source ~/catkin_ws/src/race_stack/f110_utils/scripts/pit_starter/pit_starter.sh "$ZEROTIER_IP" NUC2 rviz'


# for ease of access to the scripts
alias scripz='cd ~/catkin_ws/src/race_stack/stack_master/scripts'

# If you want to check which nucs are online
alias nucs='bash ~/catkin_ws/src/race_stack/f110_utils/scripts/nucs.sh'
```

Note that the [Pit Starter Script](../../f110_utils/scripts/pit_starter/pit_starter.sh) also supports the opening of RViz and rqt at the same time. This functionality is not included in this alias for brevity.

---
[Go back to the checklist index](./README.md)