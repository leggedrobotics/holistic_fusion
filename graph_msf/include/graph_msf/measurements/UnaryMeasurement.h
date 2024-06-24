/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_UNARYMEASUREMENT_H
#define GRAPH_MSF_UNARYMEASUREMENT_H

#include "graph_msf/measurements/Measurement.h"

namespace graph_msf {

class UnaryMeasurement : public Measurement {
 public:
  // Constructor
  UnaryMeasurement(const std::string& measurementName, const int measurementRate, const std::string& sensorFrameName,
                   const std::string& sensorFrameCorrectedName, const RobustNormEnum robustNormEnum, const double robustNormConstant,
                   const double timeStamp, const std::string& fixedFrameName, const double covarianceViolationThreshold,
                   const Eigen::Matrix<double, 6, 1>& initialSe3AlignmentNoise)
      : Measurement(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNormEnum, robustNormConstant,
                    MeasurementTypeEnum::Unary),
        timeK_(timeStamp),
        fixedFrameName_(fixedFrameName),
        covarianceViolationThreshold_(covarianceViolationThreshold),
        initialSe3AlignmentNoise_(initialSe3AlignmentNoise) {}

  // Destructor
  ~UnaryMeasurement() override = default;

  // Summary for printout
  [[nodiscard]] virtual std::string summary() const {
    std::stringstream ss;
    ss << std::endl;
    ss << "Measurement Name: " << this->measurementName() << std::endl;
    ss << "Timestamp: " << this->timeK_ << std::endl;
    ss << "Fixed Frame: " << this->fixedFrameName() << std::endl;
    ss << "Sensor Frame: " << this->sensorFrameName() << std::endl;
    return ss.str();
  }

  // Overload << operator
  friend std::ostream& operator<<(std::ostream& os, const UnaryMeasurement& unaryMeasurement) {
    os << unaryMeasurement.summary();
    return os;
  }

  // Getters
  [[nodiscard]] const std::string& fixedFrameName() const { return fixedFrameName_; }
  std::string& lv_sensorFrameName() { return sensorFrameName_; }
  [[nodiscard]] double timeK() const { return timeK_; }
  [[nodiscard]] double covarianceViolationThreshold() const { return covarianceViolationThreshold_; }
  [[nodiscard]] const Eigen::Matrix<double, 6, 1>& initialSe3AlignmentNoise() const { return initialSe3AlignmentNoise_; }
  const MeasurementTypeEnum& measurementTypeEnum() override { return measurementTypeEnum_; }

 protected:
  // Standard members
  std::string fixedFrameName_;
  double timeK_;
  double covarianceViolationThreshold_;
  Eigen::Matrix<double, 6, 1> initialSe3AlignmentNoise_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARYMEASUREMENT_H
