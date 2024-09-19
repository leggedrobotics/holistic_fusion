/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_UNARY_EXPRESSION_VELOCITY3_LOCAL_H
#define GMSF_UNARY_EXPRESSION_VELOCITY3_LOCAL_H

// GTSAM
#include <gtsam/base/types.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/expressions.h>

// Workspace
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionLocal.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"

namespace graph_msf {

class GmsfUnaryExpressionLocalVelocity3 final : public GmsfUnaryExpressionLocal<gtsam::Point3, 'd'> {
 public:
  // Constructor
  GmsfUnaryExpressionLocalVelocity3(const std::shared_ptr<UnaryMeasurementXD<Eigen::Vector3d, 3>>& velocityUnaryMeasurementPtr,
                                    const std::string& worldFrameName, const std::string& imuFrameName,
                                    const Eigen::Isometry3d& T_I_sensorFrame, const std::shared_ptr<graph_msf::ImuBuffer> imuBufferPtr)
      : GmsfUnaryExpressionLocal(velocityUnaryMeasurementPtr, worldFrameName, imuFrameName, T_I_sensorFrame),
        velocityUnaryMeasurementPtr_(velocityUnaryMeasurementPtr),
        exp_sensorFrame_v_fixedFrame_sensorFrame_(gtsam::Point3::Identity()),
        exp_R_fixedFrame_I_(gtsam::Rot3::Identity()) {
    // Find Angular Velocity in IMU Buffer which is closest to the measurement time
    // Check whether we can find an IMU measurement corresponding to the velocity measurement
    double imuTimestamp;
    graph_msf::ImuMeasurement imuMeasurement = graph_msf::ImuMeasurement();
    if (!imuBufferPtr->getClosestImuMeasurement(imuTimestamp, imuMeasurement, 0.02, velocityUnaryMeasurementPtr->timeK())) {
      REGULAR_COUT << RED_START << "No IMU Measurement (in time interval) found, assuming 0 angular velocity for the factor." << COLOR_END
                   << std::endl;
    }

    // Angular Velocity
    angularVelocity_ = imuMeasurement.angularVelocity;
  }

  // Destructor
  ~GmsfUnaryExpressionLocalVelocity3() = default;

  // Noise as GTSAM Datatype
  [[nodiscard]] const gtsam::Vector getNoiseDensity() const override {
    return velocityUnaryMeasurementPtr_->unaryMeasurementNoiseDensity();
  }

  // Return Measurement as GTSAM Datatype
  [[nodiscard]] const gtsam::Point3 getGtsamMeasurementValue() const override {
    return gtsam::Point3(velocityUnaryMeasurementPtr_->unaryMeasurement().matrix());
  }

 protected:
  // i) Generate Expression for Basic IMU State in World Frame at Key
  void generateImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) override {
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

  // iii) Transform Measurement to Core Imu Frame
  void transformImuStateToSensorFrameState() final {
    // Get relative translation
    Eigen::Vector3d I_t_I_sensorFrame = T_I_sensorFrameInit_.translation();

    // Angular velocity in the sensor frame
    // TODO: Add angular velocity bias
    gtsam::Point3_ I_w_W_I(angularVelocity_);

    // Compute Velocity of the Sensor Frame with Angular Velocity
    exp_sensorFrame_v_fixedFrame_sensorFrame_ =
        exp_sensorFrame_v_fixedFrame_sensorFrame_ + gtsam::cross(I_w_W_I, I_t_I_sensorFrame);  // I_v_W_sensorFrame at this point

    // Get Rotation from Sensor Frame to Inertial Frame
    gtsam::Rot3 R_sensorFrame_I = gtsam::Rot3(T_I_sensorFrameInit_.rotation().inverse());

    // Convert to Sensor Frame
    exp_sensorFrame_v_fixedFrame_sensorFrame_ =
        gtsam::rotate(R_sensorFrame_I, exp_sensorFrame_v_fixedFrame_sensorFrame_);  // sensorFrame_v_fixedFrame_sensorFrame
  }

  // iv) Extrinsic Calibration
  void applyExtrinsicCalibrationCorrection(const gtsam::Point3_& exp_correction) final {
    // TODO: Implement
    //    REGULAR_COUT << RED_START << "GmsfUnaryExpressionVelocity3SensorFrame: Extrinsic Calibration not implemented yet." << COLOR_END
    //                 << std::endl;
  }

  gtsam::Pose3 convertToPose3(const gtsam::Point3& measurement) final { return gtsam::Pose3(gtsam::Rot3::Identity(), measurement); }

  gtsam::Point3 convertFromPose3(const gtsam::Pose3& pose) final { return pose.translation(); }

  // Return Expression
  [[nodiscard]] const gtsam::Expression<gtsam::Point3> getGtsamExpression() const override {
    return exp_sensorFrame_v_fixedFrame_sensorFrame_;
  }

 private:
  // Full Measurement Type
  std::shared_ptr<UnaryMeasurementXD<Eigen::Vector3d, 3>> velocityUnaryMeasurementPtr_;

  // Angular Velocity
  gtsam::Point3 angularVelocity_;  // Angular Velocity

  // Expression
  gtsam::Expression<gtsam::Point3> exp_sensorFrame_v_fixedFrame_sensorFrame_;  // Translation
  gtsam::Expression<gtsam::Rot3> exp_R_fixedFrame_I_;                          // Rotation
};
}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_VELOCITY3_LOCAL_H