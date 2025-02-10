<p align="center">
<img src="docs_src/img/logo.png" width="60%" height="60%">
</p>

# Holistic Fusion: Task and Setup-agnostic Robot Localization and State Estimation with Factor Graphs

**Authors:** 
[Julian Nubert](https://www.linkedin.com/in/juliannubert/) ([nubertj@ethz.ch](mailto:nubertj@ethz.ch?subject=[GitHub])),
[Turcan Tuna](https://www.linkedin.com/in/turcantuna/)),
[Jonas Frey](https://www.linkedin.com/in/jonasfrey96/),
[Cesar Cadena](https://www.linkedin.com/in/cesar-cadena-204106b/),
[Katherine J. Kuchenbecker](https://www.linkedin.com/in/katherinekuchenbecker/),
[Shehryar Khattak](https://www.linkedin.com/in/shehryar-khattak/), 
[Marco Hutter](https://www.linkedin.com/in/marco-hutter/)

<h4>
    <a href="https://leggedrobotics.github.io/holistic_fusion/">Homepage</a> |
    <a href="https://leggedrobotics.github.io/holistic_fusion/docs">Docs</a> |
    <a href="https://leggedrobotics.github.io/holistic_fusion/doxy">Doxygen</a> |
    <a href="https://leggedrobotics.github.io/holistic_fusion/docs/examples">Examples</a> |
    <a href="https://leggedrobotics.github.io/holistic_fusion/docs/contribute">Contribute</a> |
    <a href="https://www.youtube.com/leggedrobotics">Video</a> |
</h4>

Holistic Fusion (HF) is an open-source library for flexible task and setup-agnostic robot localization and state estimation.
It provides a wide range of features that are very useful in common robotic workflows, including online sensor fusion, 
offline batch optimization and calibration. While it already supports a set of measurement factors quite common in
field robotic applications, a core purpose of HF is to simplify the process of integrating new measurement types without
manually deriving Jacobians by i) utilizing GTSAM expression factors and ii) following a specific structure to follow a 
common parent class interface, depending on the measurement type.
Currently, HF supports three general measurement types: i) absolute measurements, ii) landmark measurements, iii) local &
relative measurements.

## Resources

**[1] arXiv 2025**

* [project page](https://leggedrobotics.github.io/holistic_fusion)
* [paper](https://arxiv.org/pdf/)
* [video](https://youtu.be/leggedrobotics)

**[2] ICRA2022, Philadelphia**

* [project page](https://sites.google.com/leggedrobotics.com/gmfcl).
* [paper](https://arxiv.org/pdf/2203.01389.pdf)
* [video](https://youtu.be/syTV7Ui36jg)

## Overview

The presented framework aims for a flexible and fast fusion of multiple sensor modalities. The state estimate is
published at **imu frequency** through IMU pre-integration with a multi-threaded implementation and bookkeeping.
The measurements are added and the graph's optimization are performed in different threads.
In contrast to classical filtering-based approaches this graph-based structure also allows for a simple incorporation of
delayed sensor measurements up to the smoothingLag.

There are two intended **use-cases**:

1. Using the dual graph formulation as proposed in [1]. In this part of the implementation, there are hard-coded
   components for this specific use case.
2. A more general graph-based multi-sensor fusion. An example for fusing LiDAR odometry and IMU on the dataset of the
   [ETH Zurich Robotic Summer School](https://ethz-robotx.github.io/SuperMegaBot/) will follow shortly.

**Disclaimer:** The framework is still under development and will be updated, extended, and more generalized in the
future.

## Modules and Packages

This repository contains the following modules:

1. [Graph MSF(./graph_msf)]: The core library for the sensor fusion. This library is only dependent on Eigen and GTSAM and can be used with any communication layer (including ROS1 and ROS2).
2. [Graph MSF ROS](./ros/graph_msf_ros): This package provides an example class for using GraphMsf in ROS. It is dependant on GraphMsf and ROS.
3. [ROS1 Examples](./examples/ros): Examples on how to use GraphMsf and GraphMsfRos.
    - [](./examples/excavator_dual_graph) from [1]. This is the implementation as
      presented in the paper.
    - Single-graph standalone fusion example following soon.

## Instructions

Please refer to our [Read the Docs](https://leggedrobotics.github.io/holistic_fusion/docs) for detailed instructions.

## Paper

If you find this code useful, please consider citing:

```
@inproceedings{nubert2022graph,
  title={Graph-based Multi-sensor Fusion for Consistent Localization of Autonomous Construction Robots},
  author={Nubert, Julian and Khattak, Shehryar and Hutter, Marco},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  year={2022},
  organization={IEEE}
}
```

```
@inproceedings{nubert2022graph,
  title={Graph-based Multi-sensor Fusion for Consistent Localization of Autonomous Construction Robots},
  author={Nubert, Julian and Khattak, Shehryar and Hutter, Marco},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  year={2022},
  organization={IEEE}
}
```

## Acknowledgements

The authors thank their colleagues at ETH Z\"urich and NASA JPL for their help in conducting the robot experiments and evaluations and for using HF on their robots. 
Special thanks go to Takahiro Miki and the ANYmal Hike team at the Robotic Systems Lab (RSL), ETH Zurich, Nikita Rudin, and David Hoeller for the ANYmal Parkour experiments, Patrick Spieler for running the deployments on the JPL RACER vehicle, the entire excavation team at RSL and Gravis Robotics, Thomas Mantel and the teaching assistants of the [ETH Robotic Summer School](https://robotics-summerschool.ethz.ch/) for their help on the SuperMegaBot, and Mayank Mittal for his help in generating renderings.
