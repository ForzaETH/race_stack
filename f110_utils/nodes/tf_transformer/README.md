# TF Transformer 
Despite the name, this package does not consist of a transformer neural network implemented with tensorflow. For that look [here](https://huggingface.co/docs/transformers/main/en/index#supported-frameworks).

This package provides an odometry estimate of the F110 car purely based on [tf](http://wiki.ros.org/tf), the ROS package that "lets the user keep track of multiple coordinate frames over time".

It provides the user with an `nav_msgs/Odometry` message at the topic `tf_odom`.
The node can be launched with the launch file:
```
roslaunch tf_transformer tf_transformer 
``` 

and provides position, rotation, velocities and rotational velocities. 

---
[Go back to the utils list](../../README.md)