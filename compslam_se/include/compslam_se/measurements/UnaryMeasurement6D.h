#ifndef UNARYMEASUREMENT6D_H
#define UNARYMEASUREMENT6D_H

#include "compslam_se/measurements/UnaryMeasurement.h"

namespace compslam_se {

struct UnaryMeasurement6D : public UnaryMeasurement {
 public:
  UnaryMeasurement6D(const std::string& measurementName, const std::string& frameName, const int measurementRate, const double timeStamp,
                     const Eigen::Matrix4d& measurementPose, const Eigen::Matrix<double, 6, 1>& measurementNoise)
      : UnaryMeasurement(measurementName, frameName, measurementRate, timeStamp),
        measurementPose_(measurementPose),
        measurementNoise_(measurementNoise) {}

  const Eigen::Matrix4d& measurementPose() const { return measurementPose_; }
  const Eigen::Matrix<double, 6, 1>& measurementNoise() const { return measurementNoise_; }

 protected:
  Eigen::Matrix4d measurementPose_;
  Eigen::Matrix<double, 6, 1> measurementNoise_;
};

}  // namespace compslam_se

#endif  // UNARYMEASUREMENT_H
