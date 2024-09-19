/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_UNARY_MEASUREMENT_ABSOLUTE_H
#define GRAPH_MSF_UNARY_MEASUREMENT_ABSOLUTE_H

#include "graph_msf/measurements/UnaryMeasurement.h"

namespace graph_msf {

class UnaryMeasurementAbsolute : public virtual UnaryMeasurement {
 public:
  // Constructor
  UnaryMeasurementAbsolute(const std::string& measurementName, const int measurementRate, const std::string& sensorFrameName,
                           const std::string& sensorFrameCorrectedName, const RobustNorm robustNorm, const double timeStamp,
                           const double covarianceViolationThreshold, const std::string& fixedFrameName,
                           const Eigen::Matrix<double, 6, 1>& initialSe3AlignmentNoise)
      : UnaryMeasurement(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNorm, timeStamp,
                         covarianceViolationThreshold),
        fixedFrameName_(fixedFrameName),
        initialSe3AlignmentNoise_(initialSe3AlignmentNoise) {}

  // Destructor
  ~UnaryMeasurementAbsolute() override = default;

  // Summary for printout
  [[nodiscard]] virtual std::string summary() const {
    std::stringstream ss;
    ss << std::endl;
    ss << UnaryMeasurement::summary() << std::endl;
    ss << "Fixed Frame: " << fixedFrameName_ << std::endl;
    return ss.str();
  }

  // Overload << operator
  friend std::ostream& operator<<(std::ostream& os, const UnaryMeasurementAbsolute& unaryMeasurementAbsolute) {
    os << unaryMeasurementAbsolute.summary();
    return os;
  }

  // Getters
  [[nodiscard]] const std::string& fixedFrameName() const { return fixedFrameName_; }
  [[nodiscard]] const Eigen::Matrix<double, 6, 1>& initialSe3AlignmentNoise() const { return initialSe3AlignmentNoise_; }

 protected:
  // Standard members
  std::string fixedFrameName_;
  Eigen::Matrix<double, 6, 1> initialSe3AlignmentNoise_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARY_MEASUREMENT_ABSOLUTE_H
