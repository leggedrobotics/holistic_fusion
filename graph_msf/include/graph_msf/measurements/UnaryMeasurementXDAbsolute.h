/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_UNARYMEASUREMENTXD_ABSOLUTE_H
#define GRAPH_MSF_UNARYMEASUREMENTXD_ABSOLUTE_H

#include "graph_msf/measurements/UnaryMeasurementAbsolute.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"

namespace graph_msf {

template <class MEASUREMENT_TYPE, int DIM>
class UnaryMeasurementXDAbsolute final : public virtual UnaryMeasurementAbsolute, public virtual UnaryMeasurementXD<MEASUREMENT_TYPE, DIM> {
  static_assert(DIM > 0, "DIM must be greater than 0.");

 public:
  // Constructor
  UnaryMeasurementXDAbsolute(const std::string& measurementName, const int measurementRate, const std::string& sensorFrameName,
                             const std::string& sensorFrameCorrectedName, const RobustNorm robustNorm, const double timeStamp,
                             const double covarianceViolationThreshold, const MEASUREMENT_TYPE& unaryMeasurement,
                             const Eigen::Matrix<double, DIM, 1>& unaryMeasurementNoiseDensity, const std::string& fixedFrameName,
                             const Eigen::Matrix<double, 6, 1>& initialSe3AlignmentNoise)
      : UnaryMeasurement(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNorm, timeStamp,
                         covarianceViolationThreshold),
        UnaryMeasurementAbsolute(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNorm, timeStamp,
                                 covarianceViolationThreshold, fixedFrameName, initialSe3AlignmentNoise),
        UnaryMeasurementXD<MEASUREMENT_TYPE, DIM>(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNorm,
                                                  timeStamp, covarianceViolationThreshold, unaryMeasurement, unaryMeasurementNoiseDensity) {
  }

  // Destructor
  ~UnaryMeasurementXDAbsolute() override = default;

  // Summary for printout
  std::string summary() const override {
    std::stringstream ss;
    std::string summary = UnaryMeasurementAbsolute::summary();
    ss << summary << "Measurement: " << std::endl;  // << unaryMeasurement_ << std::endl;
    return ss.str();
  }

  // Overload << operator
  friend std::ostream& operator<<(std::ostream& os, const UnaryMeasurementXDAbsolute& unaryMeasurementXDAbsolute) {
    os << unaryMeasurementXDAbsolute.summary();
    return os;
  }
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARYMEASUREMENTXD_ABSOLUTE_H
