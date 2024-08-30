# ForzaETH Race Stack at Center for Project Based Learning

<a href="https://arxiv.org/abs/2403.11784">
    <img src="https://img.shields.io/badge/arXiv.org-2403.11784-b31b1b" alt="arXiv e-print Badge">
</a>

ForzaETH Race Stack by the [D-ITET Center for Project Based Learning (PBL)](https://pbl.ee.ethz.ch/) at ETH Zurich. 

Accompanying this repository, a paper titled *ForzaETH Race Stack - Scaled Autonomous Head-to-Head Racing on Fully Commercial off-the-Shelf Hardware* is available on [arXiv](https://arxiv.org/abs/2403.11784), detailing the system's architecture, algorithms, and performance benchmarks.

**NOTE**: For extensions on said paper, tied to specific publications, please refer to the later paragraph [Additional Publications](#additional-publications) 

## Installation

We provide an installation guide [here](./INSTALLATION.md).

## Getting started

After installation, the car (or the simulation environment) is ready to be tested. For examples on how to run the different modules on the car, refer to the [`stack_master` README](./stack_master/README.md). As a further example, the [time-trials](./stack_master/checklists/TimeTrials.md) or the [head-to-head](./stack_master/checklists/HeadToHead.md) checklists are a good starting point.

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
@misc{baumann2024forzaeth,
      title={ForzaETH Race Stack - Scaled Autonomous Head-to-Head Racing on Fully Commercial off-the-Shelf Hardware}, 
      author={Nicolas Baumann and Edoardo Ghignone and Jonas Kühne and Niklas Bastuck and Jonathan Becker and Nadine Imholz and Tobias Kränzlin and Tian Yi Lim and Michael Lötscher and Luca Schwarzenbach and Luca Tognoni and Christian Vogt and Andrea Carron and Michele Magno},
      year={2024},
      eprint={2403.11784}
}
```

## Additional Publications

### Learning-Based On-Track System Identification for Scaled Autonomous Racing in Under a Minute.
Please refer to the [`system_identification` README](./system_identification/README.md).
