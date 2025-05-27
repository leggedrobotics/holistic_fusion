# ANymal Estimator Graph

This is the readme for the GMSF-based estimator for the Anymal robot.
The estimator is based on the [GMSF](./../../README.md) library.

## Compilation

Make sure you have all dependencies for GMSF installed. This mainly also
includes having [gtsam_catkin](https://github.com/leggedrobotics/gtsam_catkin)
in your catkin workspace.

## Compilation

Compiling the code should be straightforward after GMSF compiles successfully.

```bash
catkin build anymal_estimator_graph
```

## Running the Code

### On the Robot

The code can be run using the launch file:

```bash
roslaunch anymal_estimator_graph anymal_estimator_graph.launch
```

### Replay Mode

```bash
roslaunch anymal_estimator_graph anymal_estimator_graph_replay.launch
```

### Required

Currently in the basic version an IMU topic and a LiDAR PC2 topic are required.
Both can be set in the corresponding launch file.

### Parameter Tuning

All parameters are located in the [config folder](./config).
The parameters in core alsways have to be specifiied and are robot-independent.
The anymal specific parameters are loaded in the `AnymalEstimator` main class
and can be adjusted for user needs.