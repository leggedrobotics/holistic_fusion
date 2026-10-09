/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef HOLISTIC_FUSION_HOLISTIC_H
#define HOLISTIC_FUSION_HOLISTIC_H

// Inherited Class
#include "holistic_fusion/interface/HolisticFusion.h"

// Package
#include "holistic_fusion/measurements/BinaryMeasurementXD.h"
#include "holistic_fusion/measurements/UnaryMeasurementXDAbsolute.h"
#include "holistic_fusion/measurements/UnaryMeasurementXDLandmark.h"

namespace holistic_fusion {

// Actual Class
class HolisticFusionHolistic : virtual public HolisticFusion {
 public:
  HolisticFusionHolistic();
  virtual ~HolisticFusionHolistic() = default;

  // Adder Functions for Holistic Fusion
  /// Unary Measurements
  //// Absolute Measurements: In reference frame --> systematic drift
  void addUnaryPose3AbsoluteMeasurement(const UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6>& R_T_R_S,
                                        const bool addToOnlineSmootherFlag = true) override;
  void addUnaryPosition3AbsoluteMeasurement(UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>& R_t_R_S) override;
  // Constrains the rotation about the z-axis of the world between the estimated and the measured orientation R_M_S of the sensor frame
  // in its fixed frame M. Unlike the yaw, it is defined for every attitude and leaves roll and pitch to the other measurements.
  // In a fixed frame other than the world frame, the heading goes through the alignment keyframe of that frame. Only a position or pose
  // measurement of the same frame creates that keyframe, so add it first. The heading measurement is skipped with a warning while the
  // frame has no keyframe.
  void addUnaryHeadingAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<Eigen::Matrix3d, 1>& R_M_S);
  void addUnaryVelocity3AbsoluteMeasurement(UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>& R_v_R_S) override;
  //// Local Measurements: Fully Local
  void addUnaryVelocity3LocalMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_v_F_S) override;
  //// Local Measurements: Sensor frame not rigidly attached to the IMU (e.g. a kinematic contact point).
  //// T_I_sensorFrame rotation sets the residual axes (measurement and its noise density share them),
  //// its translation is the lever arm; I_w_W_I is the angular velocity sample of the measurement.
  void addUnaryVelocity3LocalMovingFrameMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_v_F_S,
                                                    const Eigen::Isometry3d& T_I_sensorFrame, const Eigen::Vector3d& I_w_W_I);

  /// Landmark Measurements: No systematic drift
  void addUnaryPosition3LandmarkMeasurement(UnaryMeasurementXDLandmark<Eigen::Vector3d, 3>& S_t_S_L,
                                            const int landmarkCreationCounter) override;
  void addUnaryBearing3LandmarkMeasurement(UnaryMeasurementXDLandmark<Eigen::Vector3d, 3>& S_bearing_S_L) override;

  /// Binary Measurements: Purely relative
  // TODO: add binary measurements

 private:
  // Whether a measurement without a position can enter the graph, i.e. its fixed frame has an alignment keyframe. Warns if not.
  bool hasAlignmentKeyframeForMeasurementWithoutPosition_(const UnaryMeasurementAbsolute& measurement) const;
};

}  // namespace holistic_fusion

#endif  // HOLISTIC_FUSION_HOLISTIC_H
