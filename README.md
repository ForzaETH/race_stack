# ForzaETH Race Stack at Center for Project Based Learning
  
[![ROS1 Noetic](https://img.shields.io/badge/ROS-Noetic-orange.svg?logo=ros)](http://wiki.ros.org/noetic) [![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-blue.svg?logo=ros2)](http://wiki.ros.org/foxy) [![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-green.svg?logo=ros2)](http://wiki.ros.org/humble) [![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-purple.svg?logo=ros2)](http://wiki.ros.org/jazzy)
  
A modular ROS1-based software stack for scaled autonomous head-to-head racing on 1/10th-scale vehicles, built on commercial off-the-shelf hardware.

<a href="https://arxiv.org/abs/2403.11784">
    <img src="https://img.shields.io/badge/arXiv.org-2403.11784-b31b1b" alt="arXiv e-print Badge">
</a>

ForzaETH Race Stack by the [D-ITET Center for Project Based Learning (PBL)](https://pbl.ee.ethz.ch/) at ETH Zurich. 

Accompanying this repository, a paper titled *ForzaETH Race Stack - Scaled Autonomous Head-to-Head Racing on Fully Commercial off-the-Shelf Hardware* is available on [Journal of Field Robotics](https://onlinelibrary.wiley.com/doi/pdf/10.1002/rob.22429), detailing the system's architecture, algorithms, and performance benchmarks.

**NOTE**: For extensions on said paper, tied to specific publications, please refer to the later paragraph [Additional Publications](#additional-publications) 

**NOTE**: We have a **ROS2** version of this stack, check out the other branches of this repo! 

## Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Getting started](#getting-started)
- [Contributing](#contributing)
- [Acknowledgement](#acknowledgement)
- [Citing ForzaETH Race Stack](#citing-forzaeth-race-stack)
- [Additional Publications](#additional-publications)

## Installation

We provide an installation guide [here](./INSTALLATION.md).

## Quick Start

To launch a quick end-to-end simulation and run time trials:

```bash
# Start the simulator with the base system
roslaunch stack_master base_system.launch map_name:=<map> racecar_version:=NUC2 sim:=True

# Run time trials with the default controller
roslaunch stack_master time_trials.launch
```

## Getting started

After installation, the car (or the simulation environment) is ready to be tested. For examples on how to run the different modules on the car, refer to the [`stack_master` README](./stack_master/README.md). As a further example, the [time-trials](./stack_master/checklists/TimeTrials.md) or the [head-to-head](./stack_master/checklists/HeadToHead.md) checklists are a good starting point.

Or check out our [video playlist on Youtube](https://www.youtube.com/playlist?list=PLMzSGo5LtaW9cgdwHB_FnX3qlAYx7P6JI):  
<a href="https://www.youtube.com/watch?v=A9Clg1n6rII">
  <img src="./base_system/misc/install_thumbnail.png" alt="Install" style="width: 33%;"/>
</a>
<a href="https://www.youtube.com/watch?v=6PtFzrRz1GU">
  <img src="./base_system/misc/simulator_thumbnail.png" alt="Simulation" style="width: 33%;"/>
</a>
<a href="https://www.youtube.com/watch?v=ACQdLD27v-k">
  <img src="./base_system/misc/hardware_thumbnail.png" alt="Car" style="width: 33%;"/>
</a>

**Note:** Click on the thumbnails to watch the videos.


## Contributing

In case you find our package helpful and want to contribute, please either raise an issue or directly make a pull request. To create pull request please follow the guidelines in [CONTRIBUTING](./CONTRIBUTING.md).

## Acknowledgement
This project would not be possible without the use of multiple great open-sourced code bases as listed below:

- [f1tenth_system](https://github.com/f1tenth/f1tenth_system)
- [F1TENTH Racecar Simulator](https://github.com/f1tenth/f1tenth_simulator)
- [Veddar VESC Interface](https://github.com/f1tenth/vesc)
- [Cartographer](https://github.com/cartographer-project/cartographer)
- [Cartographer ROS Integration](https://github.com/cartographer-project/cartographer_ros)
- [global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)
- [RangeLibc](https://github.com/kctess5/range_libc)
- [BayesOpt4ROS](https://github.com/IntelligentControlSystems/bayesopt4ros)
- [cpu_monitor](https://github.com/alspitz/cpu_monitor)


## Citing ForzaETH Race Stack

If you found our race stack helpful in your research, we would appreciate if you cite it as follows:
```
@article{baumann2024forzaeth,
  title={ForzaETH Race Stack—Scaled Autonomous Head-to-Head Racing on Fully Commercial Off-the-Shelf Hardware},
  author={Baumann, Nicolas and Ghignone, Edoardo and K{\"u}hne, Jonas and Bastuck, Niklas and Becker, Jonathan and Imholz, Nadine and Kr{\"a}nzlin, Tobias and Lim, Tian Yi and L{\"o}tscher, Michael and Schwarzenbach, Luca and others},
  journal={Journal of Field Robotics},
  year={2024},
  publisher={Wiley Online Library}
}
```

## Additional Publications

### Learning-Based On-Track System Identification for Scaled Autonomous Racing in Under a Minute.
Please refer to the [`system_identification` README](https://github.com/ForzaETH/On-Track-SysID).

### Predictive Spliner: Data-Driven Overtaking in Autonomous Racing Using Opponent Trajectory Prediction
Please refer to the [`predictive-spliner` README](https://github.com/ForzaETH/predictive-spliner).

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
