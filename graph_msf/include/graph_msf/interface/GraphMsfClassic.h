/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_CLASSIC_H
#define GRAPH_MSF_CLASSIC_H

// Inherited Class
#include "graph_msf/interface/GraphMsf.h"
#include "graph_msf/interface/ArticulatedTurnRuntime.h"

// Package
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXDAbsolute.h"

namespace graph_msf {

// Actual Class
class GraphMsfClassic : virtual public GraphMsf {
 public:
  GraphMsfClassic();
  virtual ~GraphMsfClassic() = default;

  void configureArticulatedTurn(const ArticulatedTurnConfig& config);
  bool addArticulatedTurnSample(const ArticulatedTurnSample& sample);
  ArticulatedTurnState getArticulatedTurnState() const;
  void invalidateArticulatedTurn(const std::string& reason);
  bool addArticulatedBaselineHeading(double stamp, double heading, const Eigen::Vector3d& baseline_imu, double sigma);

  // Adder Functions for Holistic Fusion
  /// Unary Measurements
  //// Absolute Measurements
  void addUnaryRollAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& roll_F_S) override;
  void addUnaryPitchAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& pitch_F_S) override;
  void addUnaryYawAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& yaw_F_S) override;
  bool addUnaryAttitudeAbsoluteMeasurement(const UnaryMeasurementXD<Eigen::Vector3d, 2>& bodyDirectionMeasurement);

  /// Binary Measurements
  void addBinaryPose3Measurement(const BinaryMeasurementXD<Eigen::Isometry3d, 6>& F_T_F_S) override;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_CLASSIC_H
