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

/**
 * @class BinaryMeasurement
 * @brief Class to represent a binary measurement in the graph.
 *
 * Description: This class is used to represent a binary measurement in the graph.
 */
class BinaryMeasurement : public Measurement {
 public:
  /**
   * @brief Constructor to create a binary measurement.
   *
   * @copydoc Measurement::Measurement
   *
   * @param timeKm1 Time stamp of the measurement at time k-1.
   * @param timeK Time stamp of the measurement at time k.
   *
   */
  BinaryMeasurement(const std::string& measurementName, const int measurementRate, const std::string& sensorFrameName,
                    const std::string& sensorFrameCorrectedName, const RobustNorm& robustNorm, const double timeKm1, const double timeK)
      : Measurement(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNorm, MeasurementTypeEnum::Binary),
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
