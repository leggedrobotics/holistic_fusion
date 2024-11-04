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

/**
 * @class BinaryMeasurement6D
 * @brief Class to represent x-dimensional binary measurement in the graph.
 *
 * Description: This class is used to represent x-dimensional binary measurement in the graph.
 *
 * @tparam MEASUREMENT_TYPE The type of the unary measurement. E.g. Eigen::Vector3d for a 3D position measurement.
 * @tparam DIM The dimension of the unary measurement. E.g. '3' for a 3D position measurement.
 */
template <class MEASUREMENT_TYPE, int DIM>
class BinaryMeasurementXD final : public BinaryMeasurement {
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
  BinaryMeasurementXD(const std::string& measurementName, const int measurementRate, const std::string& sensorFrameName,
                      const std::string& sensorFrameCorrectedName, const RobustNorm& robustNorm, const double timeKm1, const double timeK,
                      const MEASUREMENT_TYPE& deltaMeasurement, const Eigen::Matrix<double, DIM, 1> deltaMeasurementNoiseDensity)
      : BinaryMeasurement(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNorm, timeKm1, timeK),
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
