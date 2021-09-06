# Complementrary SLAM use with rovio and leg odometry

## Dependencies

### GTSAM 4.0.3 ###
* [GTSAM 4.0.3](https://github.com/borglab/gtsam/tree/4.0.3)
* Installation flags:
    * GTSAM_BUILD_WITH_MARCH_NATIVE = OFF
    * GTSAM_POSE3_EXPMAP = ON
    * GTSAM_ROT3_EXPMAP = ON
    * GTSAM_USE_QUATERNIONS = ON
    * GTSAM_USE_SYSTEM_EIGEN = ON

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

