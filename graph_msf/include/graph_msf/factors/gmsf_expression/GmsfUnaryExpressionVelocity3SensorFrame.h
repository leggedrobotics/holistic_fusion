/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_UNARY_EXPRESSION_VELOCITY3_SENSORFRAME_H
#define GMSF_UNARY_EXPRESSION_VELOCITY3_SENSORFRAME_H

// GTSAM
#include <gtsam/base/types.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/expressions.h>

// Workspace
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpression.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"

namespace graph_msf {

class GmsfUnaryExpressionVelocity3SensorFrame final : public GmsfUnaryExpression<gtsam::Point3> {
 public:
  // Constructor
  GmsfUnaryExpressionVelocity3SensorFrame(const std::shared_ptr<UnaryMeasurementXD<Eigen::Vector3d, 3>>& velocityUnaryMeasurementPtr,
                                          const std::string& worldFrameName, const Eigen::Isometry3d& T_I_sensorFrame,
                                          const std::shared_ptr<graph_msf::ImuBuffer> imuBufferPtr)
      : GmsfUnaryExpression(velocityUnaryMeasurementPtr, worldFrameName, T_I_sensorFrame),
        velocityUnaryMeasurementPtr_(velocityUnaryMeasurementPtr),
        exp_sensorFrame_v_fixedFrame_sensorFrame_(gtsam::Point3::Identity()),
        exp_R_fixedFrame_I_(gtsam::Rot3::Identity())
  {
    // Check whether we can find an IMU measurement at the same time
    graph_msf::ImuMeasurement imuMeasurement;
    if (imuBufferPtr->getImuMeasurementAtTime(imuMeasurement, velocityUnaryMeasurementPtr_->timeK())) {
      REGULAR_COUT << "Found IMU Measurement at the same time as the velocity measurement." << std::endl;
    } else {
      REGULAR_COUT << RED_START << "No IMU Measurement found at the same time as the velocity measurement." << COLOR_END << std::endl;
    }
  }

  // Destructor
  ~GmsfUnaryExpressionVelocity3SensorFrame() override = default;

  // i) Generate Expression for Basic IMU State in World Frame at Key
  void generateExpressionForBasicImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) override {
    // Translation (core part)
    gtsam::Point3_ exp_fixedFrame_v_fixedFrame_sensorFrame_ =
        gtsam::Expression<gtsam::Vector3>(gtsam::symbol_shorthand::V(closestGeneralKey));  // W_v_W_I at this point

    // Rotation (needed for the transformation to the sensor frame)
    exp_R_fixedFrame_I_ =
        gtsam::rotation(gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKey)));  // R_W_I at this point

    // Express velocity in sensor frame
    exp_sensorFrame_v_fixedFrame_sensorFrame_ =
        gtsam::rotate(inverseRot3(exp_R_fixedFrame_I_), exp_fixedFrame_v_fixedFrame_sensorFrame_);  // I_v_W_I at this point
  }

  // Interface with three cases (non-exclusive):
  // ii) Holistically Optimize over Fixed Frames
  void transformStateFromWorldToFixedFrame(TransformsExpressionKeys& transformsExpressionKeys,
                                           const gtsam::NavState& W_currentPropagatedState,
                                           const bool centerMeasurementsAtRobotPositionBeforeAlignment) override {
    // Do nothing as this velocity measurement is expressed in the sensor frame
  }

  // iii) Transform Measurement to Core Imu Frame
  void transformStateToSensorFrame() override {
    // Get relative translation
    Eigen::Vector3d I_t_I_sensorFrame = T_I_sensorFrameInit_.translation();

    // Zero angular velocity in the sensor frame
    gtsam::Point3_ angularVelocity(gtsam::Point3::Zero());

    // Compute Velocity of the Sensor Frame with Angular Velocity
    exp_sensorFrame_v_fixedFrame_sensorFrame_ =
        exp_sensorFrame_v_fixedFrame_sensorFrame_ + gtsam::cross(angularVelocity, I_t_I_sensorFrame);  // I_v_I_sensorFrame at this point

    // Get Rotation from Sensor Frame to Inertial Frame
    gtsam::Rot3 R_sensorFrame_I = gtsam::Rot3(T_I_sensorFrameInit_.rotation().inverse());

    // Convert to Sensor Frame
    exp_sensorFrame_v_fixedFrame_sensorFrame_ =
        gtsam::rotate(R_sensorFrame_I, exp_sensorFrame_v_fixedFrame_sensorFrame_);  // sensorFrame_v_fixedFrame_sensorFrame
  }

  // iv) Extrinsic Calibration
  void addExtrinsicCalibrationCorrection(TransformsExpressionKeys& transformsExpressionKeys) override {
    // TODO: Implement
  }

  // Noise as GTSAM Datatype
  [[nodiscard]] const gtsam::Vector getNoiseDensity() const override {
    return velocityUnaryMeasurementPtr_->unaryMeasurementNoiseDensity();
  }

  // Return Measurement as GTSAM Datatype
  [[nodiscard]] const gtsam::Point3 getMeasurement() const override {
    return gtsam::Point3(velocityUnaryMeasurementPtr_->unaryMeasurement().matrix());
  }

  // Return Expression
  [[nodiscard]] const gtsam::Expression<gtsam::Point3> getExpression() const override { return exp_sensorFrame_v_fixedFrame_sensorFrame_; }

 private:
  // Full Measurement Type
  std::shared_ptr<UnaryMeasurementXD<Eigen::Vector3d, 3>> velocityUnaryMeasurementPtr_;

  // Expression
  gtsam::Expression<gtsam::Point3> exp_sensorFrame_v_fixedFrame_sensorFrame_;  // Translation
  gtsam::Expression<gtsam::Rot3> exp_R_fixedFrame_I_;                          // Rotation
};
}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_VELOCITY3_SENSORFRAME_H