/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_UNARYMEASUREMENT_H
#define GRAPH_MSF_UNARYMEASUREMENT_H

#include "graph_msf/measurements/Measurement.h"

namespace graph_msf {

struct UnaryMeasurement : public Measurement {
 public:
  UnaryMeasurement(const std::string& measurementName, const int measurementRate, const double timeStamp, const std::string& fixedFrameName,
                   const std::string& sensorFrameName, const double covarianceViolationThreshold = 0.0)
      : Measurement(measurementName, measurementRate),
        timeK_(timeStamp),
        fixedFrameName_(fixedFrameName),
        sensorFrameName_(sensorFrameName),
        covarianceViolationThreshold_(covarianceViolationThreshold) {}

  // Getters
  const std::string& fixedFrameName() const { return fixedFrameName_; }
  const std::string& sensorFrameName() const { return sensorFrameName_; }
  double timeK() const { return timeK_; }
  double covarianceViolationThreshold() const { return covarianceViolationThreshold_; }

  virtual const MeasurementTypeEnum& measurementTypeEnum() override { return measurementTypeEnum_; }

 protected:
  // Standard members
  std::string fixedFrameName_;
  std::string sensorFrameName_;
  double timeK_;
  double covarianceViolationThreshold_;
  // Enum
  MeasurementTypeEnum measurementTypeEnum_ = MeasurementTypeEnum::Unary;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARYMEASUREMENT_H
