/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_UNARY_MEASUREMENT_ABSOLUTE_H
#define GRAPH_MSF_UNARY_MEASUREMENT_ABSOLUTE_H

// Boost Optional
#include <boost/optional.hpp>

// Workspace
#include "graph_msf/measurements/UnaryMeasurement.h"

namespace graph_msf {

/**
 * @class UnaryMeasurementAbsolute
 * @brief Class to represent an absolute unary measurement in the graph. Exact dimension is not known.
 *
 * Description: This class is used to represent an absolute unary measurement in the graph. It is a base class for all absolute unary
 * measurements.
 */
class UnaryMeasurementAbsolute : public virtual UnaryMeasurement {
 public:
  /**
   * @brief Constructor to create an absolute unary measurement.
   *
   * @copydoc UnaryMeasurement::UnaryMeasurement
   *
   * @param fixedFrameName Name of the fixed frame. This is the frame w.r.t. which the ABSOLUTE measurement is defined.
   * @param worldFrameName Name of the world frame. This is needed for checking correctness of provided arguments.
   * @param initialSe3AlignmentNoise Initial noise for the SE3 alignment. If not provided, fixed frame has to be the world frame (as no
   * alignment can be performed).
   * @param se3AlignmentRandomWalk Random walk for the SE3 alignment. If not provided or set to zero, no random walk is used but alignment
   * is treated as global variable.
   */
  UnaryMeasurementAbsolute(const std::string& measurementName, const int measurementRate, const std::string& sensorFrameName,
                           const std::string& sensorFrameCorrectedName, const RobustNorm robustNorm, const double timeStamp,
                           const double covarianceViolationThreshold, const std::string& fixedFrameName, const std::string& worldFrameName,
                           const boost::optional<Eigen::Matrix<double, 6, 1>>& initialSe3AlignmentNoise = boost::none,
                           const boost::optional<Eigen::Matrix<double, 6, 1>>& se3AlignmentRandomWalk = boost::none)
      : UnaryMeasurement(measurementName, measurementRate, sensorFrameName, sensorFrameCorrectedName, robustNorm, timeStamp,
                         covarianceViolationThreshold),
        fixedFrameName_(fixedFrameName),
        worldFrameName_(worldFrameName) {
    // If initialSe3AlignmentNoise is not set, set it to zero
    if (initialSe3AlignmentNoise) {
      initialSe3AlignmentNoise_ = initialSe3AlignmentNoise.value();
    }
    // If not provided, then fixed frame has to be the world frame
    else {
      initialSe3AlignmentNoise_.setZero();
      // Check if fixed frame is world frame
      if (fixedFrameName_ != worldFrameName_) {
        throw std::runtime_error(
            "UnaryMeasurementAbsolute: If initialSe3AlignmentNoise is not provided, fixed frame has to be the world frame.");
      }
    }

    // If se3AlignmentRandomWalk is not set or zero, set it to zero
    if (se3AlignmentRandomWalk) {
      se3AlignmentRandomWalk_ = se3AlignmentRandomWalk.value();
      // If norm is not zero, set random walk flag
      if (se3AlignmentRandomWalk_.norm() > 1e-08) {
        modelAsRandomWalkFlag_ = true;
      }
    } else {
      se3AlignmentRandomWalk_.setZero();
      modelAsRandomWalkFlag_ = false;
    }
  }

  // Destructor
  ~UnaryMeasurementAbsolute() override = default;

  // Summary for printout
  [[nodiscard]] virtual std::string summary() const {
    std::stringstream ss;
    ss << std::endl;
    ss << UnaryMeasurement::summary() << std::endl;
    ss << "Fixed Frame: " << fixedFrameName_ << std::endl;
    return ss.str();
  }

  // Overload << operator
  friend std::ostream& operator<<(std::ostream& os, const UnaryMeasurementAbsolute& unaryMeasurementAbsolute) {
    os << unaryMeasurementAbsolute.summary();
    return os;
  }

  // Getters
  [[nodiscard]] const std::string& fixedFrameName() const { return fixedFrameName_; }
  [[nodiscard]] const std::string& worldFrameName() const { return worldFrameName_; }
  [[nodiscard]] const Eigen::Matrix<double, 6, 1>& initialSe3AlignmentNoise() const { return initialSe3AlignmentNoise_; }
  [[nodiscard]] const Eigen::Matrix<double, 6, 1>& se3AlignmentRandomWalk() const { return se3AlignmentRandomWalk_; }
  [[nodiscard]] bool modelAsRandomWalkFlag() const { return modelAsRandomWalkFlag_; }

 protected:
  // Standard members
  std::string fixedFrameName_;
  std::string worldFrameName_;
  Eigen::Matrix<double, 6, 1> initialSe3AlignmentNoise_;
  Eigen::Matrix<double, 6, 1> se3AlignmentRandomWalk_;
  bool modelAsRandomWalkFlag_ = false;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_UNARY_MEASUREMENT_ABSOLUTE_H
