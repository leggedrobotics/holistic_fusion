#ifndef UNARYMEASUREMENT1D_H
#define UNARYMEASUREMENT1D_H

#include "compslam_se/measurements/UnaryMeasurement.h"

namespace compslam_se {

struct UnaryMeasurement1D : public UnaryMeasurement {
 public:
  UnaryMeasurement1D(const std::string& measurementName, const std::string& frameName, const int measurementRate, const double timeStamp,
                     const double measurement, const double measurementNoise)
      : UnaryMeasurement(measurementName, frameName, measurementRate, timeStamp),
        measurement_(measurement),
        measurementNoise_(measurementNoise) {}

  const double& measurementValue() const { return measurement_; }
  const double& measurementNoise() const { return measurementNoise_; }

 protected:
  double measurement_;
  double measurementNoise_;
};

}  // namespace compslam_se

#endif  // UNARYMEASUREMENT_H
