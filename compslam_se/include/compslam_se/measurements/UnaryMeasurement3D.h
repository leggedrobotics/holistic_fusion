#ifndef UNARYMEASUREMENT3D_H
#define UNARYMEASUREMENT3D_H

#include "compslam_se/measurements/UnaryMeasurement.h"

namespace compslam_se {

struct UnaryMeasurement3D : public UnaryMeasurement {
 public:
  UnaryMeasurement3D(const std::string& measurementName, const std::string& frameName, const int measurementRate, const double timeStamp,
                     const Eigen::Vector3d& measurement, const Eigen::Vector3d& measurementNoise)
      : UnaryMeasurement(measurementName, frameName, measurementRate, timeStamp),
        measurement_(measurement),
        measurementNoise_(measurementNoise) {}

  const Eigen::Vector3d& measurementVector() const { return measurement_; }
  const Eigen::Vector3d& measurementNoise() const { return measurementNoise_; }

 protected:
  Eigen::Vector3d measurement_;
  Eigen::Vector3d measurementNoise_;
};

}  // namespace compslam_se

#endif  // UNARYMEASUREMENT_H
