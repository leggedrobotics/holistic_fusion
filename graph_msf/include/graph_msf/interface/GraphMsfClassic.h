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

// Package
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"

namespace graph_msf {

// Actual Class
class GraphMsfClassic : virtual public GraphMsf {
 public:
  GraphMsfClassic();
  virtual ~GraphMsfClassic() = default;

  // Adder Functions for Holistic Fusion
  /// Unary Measurements
  void addUnaryYawMeasurement(const UnaryMeasurementXD<double, 1>& yaw_F_S) override;

  /// Binary Measurements
  void addBinaryPose3Measurement(const BinaryMeasurementXD<Eigen::Isometry3d, 6>& F_T_F_S) override;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_CLASSIC_H
