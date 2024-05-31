/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_DELTAMEASUREMENT6D_H
#define GRAPH_MSF_DELTAMEASUREMENT6D_H

#include "graph_msf/measurements/BinaryMeasurement.h"

namespace graph_msf {

template <class MEASUREMENT_TYPE, int DIM>
class BinaryMeasurementXD final : public BinaryMeasurement {
 public:
  // Constructor
  BinaryMeasurementXD(const std::string& measurementName, const int measurementRate, const std::string& sensorFrameName,
                      const std::string& sensorFrameCorrectedName, const RobustNormEnum& robustNormEnum, const double robustNormConstant,
                      const double timeKm1, const double timeK, const MEASUREMENT_TYPE& deltaMeasurement,
                      const Eigen::Matrix<double, DIM, 1> deltaMeasurementNoiseDensity, const bool useRobustNorm = false)
      : BinaryMeasurement(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNormEnum, robustNormConstant,
                          timeKm1, timeK),
        deltaMeasurement_(deltaMeasurement),
        deltaMeasurementNoiseDensity_(deltaMeasurementNoiseDensity) {}

  // Destructor
  ~BinaryMeasurementXD() override = default;

  // Getters
  const MEASUREMENT_TYPE& deltaMeasurement() const { return deltaMeasurement_; }
  const Eigen::Matrix<double, DIM, 1>& measurementNoiseDensity() const { return deltaMeasurementNoiseDensity_; }

 protected:
  // Members
  MEASUREMENT_TYPE deltaMeasurement_;
  Eigen::Matrix<double, DIM, 1> deltaMeasurementNoiseDensity_;  // StdDev
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_DELTAMEASUREMENT_H
