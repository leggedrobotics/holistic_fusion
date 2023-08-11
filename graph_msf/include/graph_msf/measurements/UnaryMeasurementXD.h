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
                     const Eigen::Matrix<double, DIM, 1>& unaryMeasurementNoiseDensity, const double covarianceViolationThreshold = 0.0,
                     const bool useRobustNorm = false)
      : UnaryMeasurement(measurementName, measurementRate, timeStamp, fixedFrameName, sensorFrameName, covarianceViolationThreshold,
                         useRobustNorm),
        unaryMeasurement_(unaryMeasurement),
        unaryMeasurementNoiseDensity_(unaryMeasurementNoiseDensity),
        unaryMeasurementNoiseVariances_(unaryMeasurementNoiseDensity.cwiseProduct(unaryMeasurementNoiseDensity)) {}

  const MEASUREMENT_TYPE& unaryMeasurement() const { return unaryMeasurement_; }
  const Eigen::Matrix<double, DIM, 1>& unaryMeasurementNoiseDensity() const { return unaryMeasurementNoiseDensity_; }
  const Eigen::Matrix<double, DIM, 1>& unaryMeasurementNoiseVariances() const { return unaryMeasurementNoiseVariances_; }
  const Eigen::Matrix<double, DIM, DIM> unaryMeasurementNoiseCovariance() const { return unaryMeasurementNoiseVariances_.asDiagonal(); }

 protected:
  MEASUREMENT_TYPE unaryMeasurement_;
  Eigen::Matrix<double, DIM, 1> unaryMeasurementNoiseDensity_;    // StdDev
  Eigen::Matrix<double, DIM, 1> unaryMeasurementNoiseVariances_;  // Variances
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARYMEASUREMENT_H
