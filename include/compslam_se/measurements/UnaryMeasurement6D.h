#ifndef UNARYMEASUREMENT6D_H
#define UNARYMEASUREMENT6D_H

#include "compslam_se/measurements/UnaryMeasurement.h"

namespace compslam_se {

struct UnaryMeasurement6D : public UnaryMeasurement {
 public:
  explicit UnaryMeasurement6D(std::string name_, int rate_, double time_, Eigen::MatrixXd measurementPose_,
                              Eigen::MatrixXd measurementNoise_) {
    name = name_;
    rate = rate_;
    time = time_;
    measurementPose = measurementPose_;
    measurementNoise = measurementNoise_;
  }

  Eigen::Matrix4d measurementPose;
  Eigen::Matrix<double, 6, 1> measurementNoise;
};

}  // namespace compslam_se

#endif  // UNARYMEASUREMENT_H
