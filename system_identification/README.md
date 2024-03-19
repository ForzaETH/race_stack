# System Identification

This folder contains the building blocks of the system identification system at the PBL F110 Group. 

The [id_controller](./id_controller/README.md) contains scripts that automate part of the data collection for system identification.

The [id_analyser](./id_analyser/README.md) contains the scripts that analyse the data generated during the identification experiments to estimate model parameters. It also contains scripts to simulate the car's behavior and generate a lookup table to be used in conjunction with the MAP controller. 

The generated lookup tables that are to be used by the controller are copied into the cfg folder in [steering_lookup](./steering_lookup/README.md). This library then enables other ros packages to look up conversions from acceleration and speed to steering angle.
