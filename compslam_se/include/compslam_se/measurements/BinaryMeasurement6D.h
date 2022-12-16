#ifndef DELTAMEASUREMENT6D_H
#define DELTAMEASUREMENT6D_H

#include "compslam_se/measurements/BinaryMeasurement.h"

namespace compslam_se {

struct BinaryMeasurement6D : public BinaryMeasurement {
 public:
  BinaryMeasurement6D(const std::string& measurementName, const std::string& frameName, const int measurementRate, const double timeKm1,
                      const double timeK, const Eigen::Matrix4d& T_delta, const Eigen::Matrix<double, 6, 1> measurementNoise)
      : BinaryMeasurement(measurementName, frameName, measurementRate, timeKm1, timeK),
        T_delta_(T_delta),
        measurementNoise_(measurementNoise) {}

  const Eigen::Matrix4d& T_delta() const { return T_delta_; }
  const Eigen::Matrix<double, 6, 1>& measurementNoise() const { return measurementNoise_; }

 protected:
  Eigen::Matrix4d T_delta_;
  Eigen::Matrix<double, 6, 1> measurementNoise_;
};

}  // namespace compslam_se

#endif  // DELTAMEASUREMENT_H
