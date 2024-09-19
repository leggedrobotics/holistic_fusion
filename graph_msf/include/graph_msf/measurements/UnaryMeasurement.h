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
                   const std::string& sensorFrameCorrectedName, const RobustNorm robustNorm, const double timeStamp,
                   const double covarianceViolationThreshold)
      : Measurement(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNorm, MeasurementTypeEnum::Unary),
        timeK_(timeStamp),
        covarianceViolationThreshold_(covarianceViolationThreshold) {}

  // Destructor
  ~UnaryMeasurement() override = default;

  // Summary for printout
  [[nodiscard]] virtual std::string summary() const {
    std::stringstream ss;
    ss << std::endl;
    ss << "Measurement Name: " << this->measurementName() << std::endl;
    ss << std::setprecision(14) << "Timestamp: " << this->timeK_ << std::endl;
    ss << "Sensor Frame: " << this->sensorFrameName() << std::endl;
    return ss.str();
  }

  // Overload << operator
  friend std::ostream& operator<<(std::ostream& os, const UnaryMeasurement& unaryMeasurement) {
    os << unaryMeasurement.summary();
    return os;
  }

  // Getters
  std::string& lv_sensorFrameName() { return sensorFrameName_; }
  [[nodiscard]] double timeK() const { return timeK_; }
  [[nodiscard]] double covarianceViolationThreshold() const { return covarianceViolationThreshold_; }
  const MeasurementTypeEnum& measurementTypeEnum() override { return measurementTypeEnum_; }

 protected:
  // Standard members
  double timeK_;
  double covarianceViolationThreshold_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARYMEASUREMENT_H
