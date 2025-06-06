<p align="center">
<img src="docs_src/img/logo.png" width="60%" height="60%">
</p>

# Holistic Fusion: Task and Setup-agnostic Robot Localization and State Estimation with Factor Graphs

**Authors:**
[Julian Nubert](https://www.linkedin.com/in/juliannubert/) ([nubertj@ethz.ch](mailto:nubertj@ethz.ch?subject=[GitHub])),
[Turcan Tuna](https://www.linkedin.com/in/turcantuna/),
[Jonas Frey](https://www.linkedin.com/in/jonasfrey96/),
[Cesar Cadena](https://www.linkedin.com/in/cesar-cadena-204106b/),
[Katherine J. Kuchenbecker](https://www.linkedin.com/in/katherinekuchenbecker/),
[Shehryar Khattak](https://www.linkedin.com/in/shehryar-khattak/),
[Marco Hutter](https://www.linkedin.com/in/marco-hutter/)

<h4>
    <a href="https://leggedrobotics.github.io/holistic_fusion/">Homepage</a> |
    <a href="https://arxiv.org/abs/2504.06479">Paper</a> |
    <a href="https://leggedrobotics.github.io/holistic_fusion/docs">Docs</a> |
    <a href="https://leggedrobotics.github.io/holistic_fusion/doxy/html/">Doxygen</a> |
    <a href="https://leggedrobotics.github.io/holistic_fusion/docs/7_examples.html">Examples</a> |
    <a href="https://www.youtube.com/leggedrobotics">Video</a> |
</h4>

Holistic Fusion (HF) is an open-source library for flexible task and setup-agnostic robot localization and state
estimation.
It provides a wide range of features that are very useful in common robotic workflows, including online sensor fusion,
offline batch optimization and calibration. While it already supports a set of measurement factors quite common in
field robotic applications, a core purpose of HF is to simplify the process of integrating new measurement types without
manually deriving Jacobians by i) utilizing GTSAM expression factors and ii) following a specific structure to follow a
common parent class interface, depending on the measurement type.
Currently, HF supports three general measurement types: i) absolute measurements, ii) landmark measurements, iii)
local &
relative measurements.

**Disclaimer:**
The framework is still under development and will be updated, extended, and more generalized in the
future.
**More instructions will be added to the ReadTheDocs page soon.**

<p align="center">
<img src="docs_src/img/hf_illustration.png" width="100%" height="60%">
</p>

## Modules and Packages

This repository contains the following modules:

1. [Graph MSF](./graph_msf): The core library for the sensor fusion. This library depends mainly on Eigen and GTSAM and
   can be used with any communication layer (including ROS1 and ROS2).
2. [Graph MSF ROS](./ros/graph_msf_ros): This package provides an example class for GraphMsf in ROS. It is dependent on
   GraphMsf and ROS.
3. [ROS1 Examples](./examples/ros): Examples on how to use GraphMsf and GraphMsfRos.
    - [ANYmal Estimator - Quadrupedal Robot](./examples/ros/anymal_estimator_graph): This is the implementation of the
      ANYmal quadrupedal robot estimator as presented in [1], including IMU, GNSS, leg odometry, and absolute LiDAR
      measurements.
    - [HEAP - Excavator](./examples/ros/excavator_holistic_graph): This is the implementation of the HEAP excavator as
      presented in [1], including IMU, two GNSS antennas, and absolute LiDAR measurements.
    - [Grand Tour GT Generation - Leica Total Station Position & GNSS](./examples/ros/atn_position3_fuser): The GT
      generation estimator aligning two non-drifting trajectories: i) the Leica total station R3 position, and ii) the
      Novotel offline optimized SE(3) trajectory.
    - [Super Mega Bot - Robot for Teaching Purposes](./examples/ros/smb_estimator_graph): A wheeled robot integrating
      IMU, absolute LiDAR poses, and wheel encoders.
    - [Pure IMU Integration](./examples/ros/pure_imu_integration): A simple example of performing pure IMU integration
      for dead-reckoning performance testing.
    - [IMU Pose3 Fuser](./examples/ros/imu_pose3_fuser): A simple example of fusing an IMU and an SE(3) pose
      measurement.
    - [Graph MSF ROS Examples](./examples/ros/graph_msf_ros_examples): A meta-package bundling all the aforementioned
      ROS examples in one package for convenience.
4. [ROS2 Examples](./examples/ros2): COMING SOON

## Instructions

Please refer to our [Read the Docs](https://leggedrobotics.github.io/holistic_fusion/docs) for detailed instructions
regarding installation and usage.

***Note that the documentation is still under construction and will be updated soon.**

## Code Documentation

Please refer to our [Doxygen](https://leggedrobotics.github.io/holistic_fusion/doxy/html) for documentation of the code.

## Paper

If you find this code useful or use it in your work, please consider citing:

**[1] arXiv 2025**

* [Project Page](https://leggedrobotics.github.io/holistic_fusion/)
* [Paper](https://arxiv.org/abs/2504.06479)
* [Video](https://youtube.com/leggedrobotics)

```
@misc{nubert2025holisticfusiontasksetupagnostic,
      title={Holistic Fusion: Task- and Setup-Agnostic Robot Localization and State Estimation with Factor Graphs}, 
      author={Julian Nubert and Turcan Tuna and Jonas Frey and Cesar Cadena and Katherine J. Kuchenbecker and Shehryar Khattak and Marco Hutter},
      year={2025},
      eprint={2504.06479},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2504.06479}, 
}
```

**[2] ICRA2022, Philadelphia**

* [Project Page](https://sites.google.com/leggedrobotics.com/gmfcl).
* [Paper](https://arxiv.org/pdf/2203.01389.pdf)
* [Video](https://youtu.be/syTV7Ui36jg)

```
@inproceedings{nubert2022graph,
  title={Graph-based Multi-sensor Fusion for Consistent Localization of Autonomous Construction Robots},
  author={Nubert, Julian and Khattak, Shehryar and Hutter, Marco},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  year={2022},
  organization={IEEE}
}
```

## Acknowledgments

The authors thank their colleagues at ETH Zurich and NASA JPL for their help in conducting the robot experiments and
evaluations and using HF on their robots.
Special thanks go to Takahiro Miki and the ANYmal Hike team at the Robotic Systems Lab (RSL), ETH Zurich, Nikita Rudin,
and David Hoeller for the ANYmal Parkour experiments, Patrick Spieler for running the deployments on the JPL RACER
vehicle, the entire excavation team at RSL and Gravis Robotics, Thomas Mantel and the teaching assistants of
the [ETH Robotic Summer School](https://robotics-summerschool.ethz.ch/) for their help on the SuperMegaBot, and Mayank
Mittal for his help in generating renderings.
