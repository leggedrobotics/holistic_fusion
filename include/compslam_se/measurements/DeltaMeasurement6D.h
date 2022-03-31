#ifndef DELTAMEASUREMENT6D_H
#define DELTAMEASUREMENT6D_H

#include "compslam_se/measurements/DeltaMeasurement.h"

namespace compslam_se {

struct DeltaMeasurement6D : public DeltaMeasurement {
 public:
  DeltaMeasurement6D(std::string name_, int rate_, double timeKm1_, double timeK_, Eigen::MatrixXd T_delta_,
                     Eigen::MatrixXd measurementNoise_) {
    name = name_;
    rate = rate_;
    timeKm1 = timeKm1_;
    time = timeK_;
    T_delta = T_delta_;
    measurementNoise = measurementNoise_;
  }

  Eigen::Matrix4d T_delta;
  Eigen::Matrix<double, 6, 1> measurementNoise;
};

}  // namespace compslam_se

#endif  // DELTAMEASUREMENT_H
