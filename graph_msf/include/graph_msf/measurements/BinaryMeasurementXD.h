/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_DELTAMEASUREMENT6D_H
#define GRAPH_MSF_DELTAMEASUREMENT6D_H

#include "graph_msf/measurements/BinaryMeasurement.h"

namespace graph_msf {

template <class MEASUREMENT_TYPE, int DIM>
struct BinaryMeasurementXD : public BinaryMeasurement {
 public:
  BinaryMeasurementXD(const std::string& measurementName, const int measurementRate, const double timeKm1, const double timeK,
                      const std::string& sensorFrameName, const MEASUREMENT_TYPE& deltaMeasurement,
                      const Eigen::Matrix<double, DIM, 1> deltaMeasurementNoise)
      : BinaryMeasurement(measurementName, measurementRate, timeKm1, timeK, sensorFrameName),
        deltaMeasurement_(deltaMeasurement),
        deltaMeasurementNoise_(deltaMeasurementNoise) {}

  const MEASUREMENT_TYPE& deltaMeasurement() const { return deltaMeasurement_; }
  const Eigen::Matrix<double, DIM, 1>& measurementNoise() const { return deltaMeasurementNoise_; }

 protected:
  MEASUREMENT_TYPE deltaMeasurement_;
  Eigen::Matrix<double, DIM, 1> deltaMeasurementNoise_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_DELTAMEASUREMENT_H
