# Examples

## excavator_dual_graph

### Visualization
For a visualization of a simplified model of our excavator HEAP clone the following repository into the workspace:
```bash
https://github.com/leggedrobotics/rsl_heap.git
```
This step can also be left out.

### Compiling the Code
This is the example of the code of our ICRA2022 paper [1].
For compiling this code simply run:
```bash
catkin build excavator_dual_graph
```

### Data
For two example scenarios on our test-field at ETH Zurich please download the data located at the
[Google Drive Link](https://drive.google.com/drive/folders/1qZg_DNH3wXnQu4tNIcqY925KZFDu8y0M?usp=sharing).

### Running the Code
In terminal 1 run (after sourcing the workspace):
```bash
roslaunch excavator_dual_graph dual_graph.launch
```
In terminal 2 play one of the rosbags located at the provided link location. Make sure to set the `--clock`-flag.
Either
```bash
rosbag play 1_digging_filtered.bag --clock
# or
rosbag play 2_driving_away_filtered.bag --clock
```