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
#include "graph_msf/measurements/UnaryMeasurementXDAbsolute.h"

namespace graph_msf {

// Actual Class
class GraphMsfHolistic : virtual public GraphMsf {
 public:
  GraphMsfHolistic();
  virtual ~GraphMsfHolistic() = default;

  // Adder Functions for Holistic Fusion
  /// Unary Measurements
  //// Absolute Measurements: In reference frame --> systematic drift
  void addUnaryPose3AbsoluteMeasurement(const UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6>& R_T_R_S) override;
  void addUnaryPosition3AbsoluteMeasurement(UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>& R_t_R_S) override;
  void addUnaryVelocity3AbsoluteMeasurement(UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>& R_v_R_S) override;
  void addUnaryRollAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& roll_R_S) override;
  void addUnaryPitchAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& pitch_R_S) override;
  //// Local Measurements: Fully Local
  void addUnaryVelocity3LocalMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_v_F_S) override;

  /// Landmark Measurements: No systematic drift
  void addUnaryPosition3LandmarkMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_t_S_L) override;
  void addUnaryBearing3LandmarkMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_bearing_S_L) override;

  /// Binary Measurements: Purely relative
  // TODO: add binary measurements
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_HOLISTIC_H
