# Frenet Conversion Library

It constitutes a C library to compute the frenet conversion. 
It furthermore contains the definition of the frenet conversion service, which is instead at `<race_stack folder>/f110_utils/nodes/frenet_conversion_server`.

## Basic usage

### C++
An example of the usage of the frenet conversion library can be seen in the Frenet Odometry Republisher. It basically imports the library in the header file (`<race_stack folder>/f110_utils/nodes/frenet_odom_republisher/include/frenet_odom_republisher_node.h`, line 11) then it instantiates the converter object in line 32.

### Python 
TODO: We should write a wrapper/version of the converter in python

---
[Go back to the utils list](../../README.md)