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
                   const RobustNormEnum robustNormEnum, const double robustNormConstant, const double timeStamp,
                   const std::string& fixedFrameName, const double covarianceViolationThreshold = 0.0)
      : Measurement(measurementName, measurementRate, sensorFrameName, robustNormEnum, robustNormConstant, MeasurementTypeEnum::Unary),
        timeK_(timeStamp),
        fixedFrameName_(fixedFrameName),
        covarianceViolationThreshold_(covarianceViolationThreshold) {}

  // Destructor
  ~UnaryMeasurement() override = default;

  // Getters
  [[nodiscard]] const std::string& fixedFrameName() const { return fixedFrameName_; }
  [[nodiscard]] const std::string& sensorFrameName() const { return sensorFrameName_; }
  std::string& lv_sensorFrameName() { return sensorFrameName_; }
  [[nodiscard]] double timeK() const { return timeK_; }
  [[nodiscard]] double covarianceViolationThreshold() const { return covarianceViolationThreshold_; }

  const MeasurementTypeEnum& measurementTypeEnum() override { return measurementTypeEnum_; }

 protected:
  // Standard members
  std::string fixedFrameName_;
  double timeK_;
  double covarianceViolationThreshold_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARYMEASUREMENT_H
