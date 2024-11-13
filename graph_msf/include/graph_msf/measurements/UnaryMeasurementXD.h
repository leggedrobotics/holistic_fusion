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

/**
 * @class UnaryMeasurementXD
 * @brief Class to represent x-dimensional unary measurement in the graph.
 *
 * Description: This class is used to represent x-dimensional unary measurement in the graph.
 *
 * @tparam MEASUREMENT_TYPE The type of the unary measurement. E.g. Eigen::Vector3d for a 3D position measurement.
 * @tparam DIM The dimension of the unary measurement. E.g. '3' for a 3D position measurement.
 */
template <class MEASUREMENT_TYPE, int DIM>
class UnaryMeasurementXD : public virtual UnaryMeasurement {
 public:
  /**
   * @brief Constructor to create an absolute unary measurement.
   *
   * @copydoc UnaryMeasurement::UnaryMeasurement
   *
   * @param unaryMeasurement The templated unary measurement.
   * @param unaryMeasurementNoiseDensity The noise density of the unary measurement.
   *
   */
  UnaryMeasurementXD(const std::string& measurementName, const int measurementRate, const std::string& sensorFrameName,
                     const std::string& sensorFrameCorrectedName, const RobustNorm robustNorm, const double timeStamp,
                     const double covarianceViolationThreshold, const MEASUREMENT_TYPE& unaryMeasurement,
                     const Eigen::Matrix<double, DIM, 1>& unaryMeasurementNoiseDensity)
      : UnaryMeasurement(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNorm, timeStamp,
                         covarianceViolationThreshold),
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
