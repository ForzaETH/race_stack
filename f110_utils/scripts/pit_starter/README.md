# Pit starter 
The content of this folder can be used to automatically setup your laptop to see a roscore running on a car. 

## Prerequisites
Your computer should be in the ZeroTier network. to do that consult the [networking guide](../../../stack_master/checklists/networking.md). 
## How to modify the script to use your own fixed IP
Open the [pit_starter.sh](./pit_starter.sh) file and write in the point marked with the todo the fixed IP of the car you want to connect to.
 
## How to connect the pit
On your pit laptop.
- `cd <race_stack folder>/f110_utils/scripts/pit_starter`
- `source pit_starter.sh <YOUR_ZEROTIER_IP> <NUC3 or NUC4> rviz`
- Enjoy freshly sourced NUCX. The script runs the rviz with the correct config.
- Pro tip: If you only want to source the NUCX then dont use the `rviz` argument


---
[Go back to the utils list](../../README.md)