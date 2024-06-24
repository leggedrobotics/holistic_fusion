/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_UNARYMEASUREMENTXD_H
#define GRAPH_MSF_UNARYMEASUREMENTXD_H

#include "graph_msf/measurements/UnaryMeasurement.h"

namespace graph_msf {

template <class MEASUREMENT_TYPE, int DIM>
class UnaryMeasurementXD final : public UnaryMeasurement {
 public:
  // Constructor
  UnaryMeasurementXD(const std::string& measurementName, const int measurementRate, const std::string& sensorFrameName,
                     const std::string& sensorFrameCorrectedName, const RobustNormEnum robustNormEnum, const double robustNormConstant,
                     const double timeStamp, const std::string& fixedFrameName, const double covarianceViolationThreshold,
                     const Eigen::Matrix<double, 6, 1>& initialSe3AlignmentNoise, const MEASUREMENT_TYPE& unaryMeasurement,
                     const Eigen::Matrix<double, DIM, 1>& unaryMeasurementNoiseDensity)
      : UnaryMeasurement(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNormEnum, robustNormConstant,
                         timeStamp, fixedFrameName, covarianceViolationThreshold, initialSe3AlignmentNoise),
        unaryMeasurement_(unaryMeasurement),
        unaryMeasurementNoiseDensity_(unaryMeasurementNoiseDensity),
        unaryMeasurementNoiseVariances_(unaryMeasurementNoiseDensity.cwiseProduct(unaryMeasurementNoiseDensity)) {}

  // Destructor
  ~UnaryMeasurementXD() override = default;

  // Summary for printout
  std::string summary() const override {
    std::stringstream ss;
    std::string summary = UnaryMeasurement::summary();
    ss << summary << "Measurement: " << std::endl;  // << unaryMeasurement_ << std::endl;
    return ss.str();
  }

  // Overload << operator
  friend std::ostream& operator<<(std::ostream& os, const UnaryMeasurementXD& unaryMeasurementXD) {
    os << unaryMeasurementXD.summary();
    return os;
  }

  // Accessors
  const MEASUREMENT_TYPE& unaryMeasurement() const { return unaryMeasurement_; }
  MEASUREMENT_TYPE& unaryMeasurement() { return unaryMeasurement_; }
  const Eigen::Matrix<double, DIM, 1>& unaryMeasurementNoiseDensity() const { return unaryMeasurementNoiseDensity_; }
  const Eigen::Matrix<double, DIM, 1>& unaryMeasurementNoiseVariances() const { return unaryMeasurementNoiseVariances_; }
  const Eigen::Matrix<double, DIM, DIM> unaryMeasurementNoiseCovariance() const { return unaryMeasurementNoiseVariances_.asDiagonal(); }

  // Modifiers
  MEASUREMENT_TYPE& lv_unaryMeasurement() { return unaryMeasurement_; }

 protected:
  MEASUREMENT_TYPE unaryMeasurement_;
  Eigen::Matrix<double, DIM, 1> unaryMeasurementNoiseDensity_;    // StdDev
  Eigen::Matrix<double, DIM, 1> unaryMeasurementNoiseVariances_;  // Variances
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARYMEASUREMENTXD_H
