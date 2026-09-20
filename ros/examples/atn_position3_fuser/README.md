# atn_leica_position_graph
[Thesis](https://1drv.ms/b/s!AsodIbzyjoMbjeIQAJcPEIf3MWrLWA?e=4CjaUr): "Factor graph based state estimation for UAVs in construction environments"

# Installation
Install [ROS noetic](https://wiki.ros.org/noetic/Installation/Ubuntu).

Install gtsam & graph_msf by following [these installation instructions](https://leggedrobotics.github.io/holistic_fusion/docs/1_installation.html).
This package is part of the [holistic_fusion](https://github.com/leggedrobotics/holistic_fusion) repository (`ros/examples/atn_position3_fuser`).
```
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin init
cd src
git clone https://github.com/leggedrobotics/holistic_fusion.git
git clone https://github.com/aithon-robotics/atn_leica_timesync.git
cd ..
catkin build
```

# Launch
To start atn_leica_position_graph sensor fusion two Terminals are needed:
```
source ~/catkin_ws/devel/setup.bash
roslaunch atn_leica_position_graph graph_leica.launch
```

```
source ~/catkin_ws/devel/setup.bash
rosrun atn_leica_timesync atn_leica_timesync_node 
```

# Related packages

| Package         | Description                     | Link                                                           |
| --------------- | ------------------------------- | -------------------------------------------------------------- |
| atn_leica_timesync| Timesynchronization of total station & system clock | [atn_leica_timesync](https://github.com/aithon-robotics/atn_leica_timesync)         |
| graph_msf        | Factor graph framework this package is based on     | [holistic_fusion](https://github.com/leggedrobotics/holistic_fusion) |

