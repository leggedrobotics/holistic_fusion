/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_DELTAMEASUREMENT_H
#define GRAPH_MSF_DELTAMEASUREMENT_H

#include "graph_msf/measurements/Measurement.h"

namespace graph_msf {

class BinaryMeasurement : public Measurement {
 public:
  // Constructor
  BinaryMeasurement(const std::string& measurementName, const int measurementRate, const std::string& sensorFrameName,
                    const std::string& sensorFrameCorrectedName, const RobustNormEnum& robustNormEnum, const double robustNormConstant,
                    const double timeKm1, const double timeK)
      : Measurement(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNormEnum, robustNormConstant,
                    MeasurementTypeEnum::Binary),
        timeKm1_(timeKm1),
        timeK_(timeK) {}

  // Destructor
  ~BinaryMeasurement() override = default;

  // Getters
  [[nodiscard]] double timeKm1() const { return timeKm1_; }
  [[nodiscard]] double timeK() const { return timeK_; }

  const MeasurementTypeEnum& measurementTypeEnum() override { return measurementTypeEnum_; }

 protected:
  // Standard members
  double timeKm1_;
  double timeK_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_DELTAMEASUREMENT_H
