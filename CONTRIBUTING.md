# Contributing

## We welcome contributions from the community and are happy to have them. Please follow this guide when logging issues or making changes.

## ROS Packages

### Package Structure 
TODO: should we encourage more packages, or keep the current structure (which is quite random)?

### General Code guidelines
- Add TODO when a minor code smell is found, e.g. a hard-coded number that should be a parameter. (You are then welcome to fix them and propose the fixes in a pull request!).
- Add docstrings to the code. To ease the setup in VSCode you can use the `Python Docstring Generator` extension (https://marketplace.visualstudio.com/items?itemName=njpwerner.autodocstring).
- Add comments expecially for the parts of the code that are not straightforward, or to explain why a certain approach was chosen.
- In Python, use type-hinting for the parameters and the return values of the functions, when it's not obvious (e.g. init function and callback functions).
- remove all commented code! Also unused imports and variables should be removed. 

### ROS nodes
- use a global parameter from the context list below to specify the context in which the node is running instead of hard-coding it. If a new context is needed, open a PR to add it to the list.
- Context list:
  - `/sim`: the node is running in the simulator (if `False`, the node is running on the car)
  - `/measure`: the node is running with additional measurements 
  - `/from_bag`: the node is running from a rosbag
  - `/racecar_version`: the node is assuming the racecar to be <racecar_version>
- Use ROS logging functions to log messages, with the default logging level set to `INFO`.


An example is in the [`planner/spliner/src/spliner_node.py`](./planner/spliner/src/spliner_node.py) file.

### ROS launch files
- document the arguments of the launch file with the `doc` attribute of the `arg` tag where reasonabele (e.g. when the arguments are limited to a set of few values)
- keep formatting consistent, by using the VSCode plugin for `XML` (https://marketplace.visualstudio.com/items?itemName=redhat.vscode-xml)

### README.md
Make sure the readme of a package/core node is up to date and contains the following sections:
- Description
- Parameters
- Input/Output Topic Signature
- TODO: License?

An example is in the [`planner/spliner/README.md`](./planner/spliner/README.md) file.
