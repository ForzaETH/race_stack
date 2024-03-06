# Id Analyser 
This packages contains the scripts to conduct the data analysis for system identifying the F110 car at PBL
More info about the System Identification procedure are [here](../../stack_master/checklists/SysID.md).

## Getting Started

For the analyser you need bagpy, as bagpy only works in python 3, but you might be running python2, create a virtual environment following this guide:
```
https://code.visualstudio.com/docs/python/environments
```
which boils down to doing the following <strong>in the id_analyser folder</strong>:
```shell
sudo apt-get install python3-venv
# move to the current folder 
cd /home/icra_crew/catkin_ws/src/race_stack/system_identification/id_analyser/
# create a new virtual environment
python3 -m venv .venv
```
Then install the python extension for vscode and open the command palete using ```Ctrl+Shift+P``` and search for
"Python: Select Interpreter". If the interpreter does not show up, just manually select it (.venv/bin/python)

Then install bagpy and anything else that might be required.

```shell
source /home/icra_crew/catkin_ws/src/race_stack/system_identification/id_analyser/.venv/bin/activate 
pip install wheel importlib-resources tk bagpy 
```

## Physical System Identification

Parameters that need to be identified physically are:
- I_z: Moment of Inertia around yaw axis - Bifilar Pendulum experiment
- h_cg: Height of the center of gravity over the wheel axes
- l_f: distance of front axis from center of mass
- l_r: distance of rear axis from center of mass
- l_wb: wheelbase, should be l_r + l_f
- m: mass of the model

Typically these don't change significantly and need to be identified only once. If a new model is added, generate a model file using the script [add_model](add_model.py), where you can specify which tire model it is supposed to use. Prefer pacejka if you can perform a system identification.

## Data Driven Identification

After obtaining a physical model, we are still missing a few crucial model parameters, which can be obtained through a data driven approach. Refer to the [checklist](../../stack_master/checklists/SysID.md). for an extensive guide.