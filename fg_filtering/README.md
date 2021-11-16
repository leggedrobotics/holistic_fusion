# Complementrary SLAM use with rovio and leg odometry

## Dependencies

* [GTSAM 4.0.3](https://github.com/borglab/gtsam/tree/4.0.3)
  * ```git clone git@github.com:borglab/gtsam.git```
  * ```git checkout 4.0.3```
  * ```mkdir build && cd build```
  * Local installation:```cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME/software/install ..```
  * Install ccmake: ```sudo apt-get install cmake-curses-gui```
  * ```ccmake .```
  * Set following flags to:
    * GTSAM_BUILD_WITH_MARCH_NATIVE = OFF
    * GTSAM_POSE3_EXPMAP = ON
    * GTSAM_ROT3_EXPMAP = ON
    * GTSAM_USE_QUATERNIONS = ON
    * GTSAM_USE_SYSTEM_EIGEN = ON
  * configure previous settings and then run ```make -j12```
  * ```make install```
  * Before using your code add the following to your .bashrc-file:
    * ```export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$HOME/software/install```
    * ```export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/software/install```
* Needs external LiDAR odometry:
  * Compslam
    * In catkin workspace:
      * ```git clone git@bitbucket.org:leggedrobotics/compslam.git```
      * ```git checkout dev/menzi```
## Build workspace
* ```catkin build loam```
* ```catkin build fg_filtering```
* ```catkin build m545_state_estimation_sim```
## Launch state estimator:
* ```roslaunch m545_state_estimation_sim m545.launch```

## Parameters
### Lord microstrain MV5-AR  IMU in Roofbox
* [Datasheet](https://www.microstrain.com/sites/default/files/mv5-ar_datasheet_8400-0122_rev_d.pdf)
* Values
  * Noise density
    * Accelerometer: 85 ug/sqrt(Hz)=85e-5m/s^2/sqrt(Hz)
    * Gyroscope: 0.0075° /sec/√Hz=1.309e-4 rad/s/sqrt(Hz)

### Who do I talk to? ###

* Julian Nubert (nubertj@ethz.ch)
* Shehryar Khattak

