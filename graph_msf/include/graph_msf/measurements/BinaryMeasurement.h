/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_DELTAMEASUREMENT_H
#define GRAPH_MSF_DELTAMEASUREMENT_H

#include "graph_msf/measurements/Measurement.h"

namespace graph_msf {

struct BinaryMeasurement : public Measurement {
 public:
  BinaryMeasurement(const std::string& measurementName, const int measurementRate, const double timeKm1, const double timeK,
                    const std::string& sensorFrameName, const bool useRobustNorm = false)
      : Measurement(measurementName, measurementRate, useRobustNorm), timeKm1_(timeKm1), timeK_(timeK), sensorFrameName_(sensorFrameName) {}

  // Getters
  double timeKm1() const { return timeKm1_; }
  double timeK() const { return timeK_; }
  const std::string& sensorFrameName() const { return sensorFrameName_; }

  virtual const MeasurementTypeEnum& measurementTypeEnum() override { return measurementTypeEnum_; }

 protected:
  // Standard members
  std::string sensorFrameName_;
  double timeKm1_;
  double timeK_;
  // Enum
  MeasurementTypeEnum measurementTypeEnum_ = MeasurementTypeEnum::Binary;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_DELTAMEASUREMENT_H
