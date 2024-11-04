/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_UNARY_MEASUREMENT_XD_LANDMARK_H
#define GRAPH_MSF_UNARY_MEASUREMENT_XD_LANDMARK_H

// Boost Optional
#include <boost/optional.hpp>

// Workspace
#include "graph_msf/measurements/UnaryMeasurementLandmark.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"

namespace graph_msf {

/**
 * @class UnaryMeasurementXDLandmark
 * @brief Class to represent x-dimensional landmark unary measurement in the graph.
 *
 * Description: This class is used to represent x-dimensional absolute unary measurement in the graph.
 *
 * @tparam MEASUREMENT_TYPE The type of the unary measurement. E.g. Eigen::Vector3d for a 3D position measurement.
 * @tparam DIM The dimension of the unary measurement. E.g. '3' for a 3D position measurement.
 */
template <class MEASUREMENT_TYPE, int DIM>
class UnaryMeasurementXDLandmark final : public virtual UnaryMeasurementLandmark, public virtual UnaryMeasurementXD<MEASUREMENT_TYPE, DIM> {
  static_assert(DIM > 0, "DIM must be greater than 0.");

 public:
  /**
   * @brief Constructor to create an absolute unary measurement with a specific dimension.
   *
   * @copydoc UnaryMeasurementAbsolute::UnaryMeasurementAbsolute
   *
   */
  UnaryMeasurementXDLandmark(const std::string& measurementName, const int measurementRate, const std::string& sensorFrameName,
                             const std::string& sensorFrameCorrectedName, const RobustNorm robustNorm, const double timeStamp,
                             const double covarianceViolationThreshold, const MEASUREMENT_TYPE& unaryMeasurement,
                             const Eigen::Matrix<double, DIM, 1>& unaryMeasurementNoiseDensity, const std::string& worldFrameName)
      : UnaryMeasurement(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNorm, timeStamp,
                         covarianceViolationThreshold),
        UnaryMeasurementLandmark(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNorm, timeStamp,
                                 covarianceViolationThreshold, worldFrameName),
        UnaryMeasurementXD<MEASUREMENT_TYPE, DIM>(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNorm,
                                                  timeStamp, covarianceViolationThreshold, unaryMeasurement, unaryMeasurementNoiseDensity) {
  }

  // Destructor
  ~UnaryMeasurementXDLandmark() override = default;

  // Summary for printout
  std::string summary() const override {
    std::stringstream ss;
    std::string summary = UnaryMeasurementLandmark::summary();
    ss << summary << "Measurement: " << std::endl;  // << unaryMeasurement_ << std::endl;
    return ss.str();
  }

  // Overload << operator
  friend std::ostream& operator<<(std::ostream& os, const UnaryMeasurementXDLandmark& unaryMeasurementXDLandmark) {
    os << unaryMeasurementXDLandmark.summary();
    return os;
  }
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARY_MEASUREMENT_XD_LANDMARK_H
