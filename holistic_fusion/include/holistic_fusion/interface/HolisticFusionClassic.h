/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef HOLISTIC_FUSION_CLASSIC_H
#define HOLISTIC_FUSION_CLASSIC_H

// Inherited Class
#include "holistic_fusion/interface/HolisticFusion.h"

// Package
#include "holistic_fusion/measurements/BinaryMeasurementXD.h"
#include "holistic_fusion/measurements/UnaryMeasurementXDAbsolute.h"

namespace holistic_fusion {

// Actual Class
class HolisticFusionClassic : virtual public HolisticFusion {
 public:
  HolisticFusionClassic();
  virtual ~HolisticFusionClassic() = default;

  // Adder Functions for Holistic Fusion
  /// Unary Measurements
  //// Absolute Measurements
  void addUnaryRollAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& roll_F_S) override;
  void addUnaryPitchAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& pitch_F_S) override;
  // Constrains the yaw of the sensor frame in the world frame, independent of the measurement's fixed frame
  void addUnaryYawAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& yaw_W_S) override;

  /// Binary Measurements
  void addBinaryPose3Measurement(const BinaryMeasurementXD<Eigen::Isometry3d, 6>& F_T_F_S) override;
};

}  // namespace holistic_fusion

#endif  // HOLISTIC_FUSION_CLASSIC_H
