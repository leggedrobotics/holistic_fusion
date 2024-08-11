/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_HOLISTIC_H
#define GRAPH_MSF_HOLISTIC_H

// Inherited Class
#include "graph_msf/interface/GraphMsf.h"

// Package
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"

namespace graph_msf {

// Actual Class
class GraphMsfHolistic : virtual public GraphMsf {
 public:
  GraphMsfHolistic();
  virtual ~GraphMsfHolistic() = default;

  // Adder Functions for Holistic Fusion
  /// Unary Measurements
  void addUnaryPose3Measurement(const UnaryMeasurementXD<Eigen::Isometry3d, 6>& F_T_F_S) override;
  void addUnaryPosition3Measurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& F_t_F_S) override;
  void addUnaryVelocity3FixedFrameMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& F_v_F_S) override;
  void addUnaryVelocity3SensorFrameMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_v_F_S) override;
  void addUnaryRollMeasurement(const UnaryMeasurementXD<double, 1>& roll_F_S) override;
  void addUnaryPitchMeasurement(const UnaryMeasurementXD<double, 1>& pitch_F_S) override;

  /// Binary Measurements
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_HOLISTIC_H
