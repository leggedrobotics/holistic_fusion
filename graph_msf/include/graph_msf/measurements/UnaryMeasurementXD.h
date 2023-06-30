/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_UNARYMEASUREMENT6D_H
#define GRAPH_MSF_UNARYMEASUREMENT6D_H

#include "graph_msf/measurements/UnaryMeasurement.h"

namespace graph_msf {

template <class MEASUREMENT_TYPE, int DIM>
struct UnaryMeasurementXD : public UnaryMeasurement {
 public:
  UnaryMeasurementXD(const std::string& measurementName, const int measurementRate, const double timeStamp,
                     const std::string& fixedFrameName, const std::string& sensorFrameName, const MEASUREMENT_TYPE& unaryMeasurement,
                     const Eigen::Matrix<double, DIM, 1>& unaryMeasurementNoise)
      : UnaryMeasurement(measurementName, measurementRate, timeStamp, fixedFrameName, sensorFrameName),
        unaryMeasurement_(unaryMeasurement),
        unaryMeasurementNoise_(unaryMeasurementNoise) {}

  const MEASUREMENT_TYPE& unaryMeasurement() const { return unaryMeasurement_; }
  const Eigen::Matrix<double, DIM, 1>& unaryMeasurementNoise() const { return unaryMeasurementNoise_; }

 protected:
  MEASUREMENT_TYPE unaryMeasurement_;
  Eigen::Matrix<double, DIM, 1> unaryMeasurementNoise_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARYMEASUREMENT_H
