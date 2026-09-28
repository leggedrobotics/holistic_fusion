/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef HOLISTIC_FUSION_UNARY_MEASUREMENT_LANDMARK_H
#define HOLISTIC_FUSION_UNARY_MEASUREMENT_LANDMARK_H

// Workspace
#include "holistic_fusion/measurements/UnaryMeasurement.h"

namespace holistic_fusion {

/**
 * @class UnaryMeasurementAbsolute
 * @brief Class to represent an absolute unary measurement in the graph. Exact dimension is not known.
 *
 * Description: This class is used to represent an absolute unary measurement in the graph. It is a base class for all absolute unary
 * measurements.
 */
class UnaryMeasurementLandmark : public virtual UnaryMeasurement {
 public:
  /**
   * @brief Constructor to create a landmark unary measurement.
   *
   * @copydoc UnaryMeasurement::UnaryMeasurement
   *
   * @param worldFrameName Name of the world frame. In current implementation, each landmark is defined w.r.t. the world frame.
   *
   */
  UnaryMeasurementLandmark(const std::string& measurementName, const int measurementRate, const std::string& sensorFrameName,
                           const std::string& sensorFrameCorrectedName, const RobustNorm robustNorm, const double timeStamp,
                           const double covarianceViolationThreshold, const std::string& worldFrameName)
      : UnaryMeasurement(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNorm, timeStamp,
                         covarianceViolationThreshold),
        worldFrameName_(worldFrameName) {}

  // Destructor
  ~UnaryMeasurementLandmark() override = default;

  // Summary for printout
  [[nodiscard]] virtual std::string summary() const {
    std::stringstream ss;
    ss << std::endl;
    ss << UnaryMeasurement::summary() << std::endl;
    ss << "World Frame: " << worldFrameName_ << std::endl;
    return ss.str();
  }

  // Overload << operator
  friend std::ostream& operator<<(std::ostream& os, const UnaryMeasurementLandmark& unaryMeasurementAbsolute) {
    os << unaryMeasurementAbsolute.summary();
    return os;
  }

  // Getters
  [[nodiscard]] const std::string& worldFrameName() const { return worldFrameName_; }

 protected:
  // Standard members
  std::string worldFrameName_;
};

}  // namespace holistic_fusion

#endif  // HOLISTIC_FUSION_UNARY_MEASUREMENT_LANDMARK_H
