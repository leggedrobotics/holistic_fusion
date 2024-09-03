/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_UNARY_EXPRESSION_POSITION3_LANDMARK_H
#define GMSF_UNARY_EXPRESSION_POSITION3_LANDMARK_H

// GTSAM
#include <gtsam/base/types.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/expressions.h>
#include "gtsam/navigation/NavState.h"

// Workspace
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionLandmark.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"

namespace graph_msf {

class GmsfUnaryExpressionLandmarkPosition3 final : public GmsfUnaryExpressionLandmark<gtsam::Point3, 'd'> {
 public:
  // Constructor
  GmsfUnaryExpressionLandmarkPosition3(const std::shared_ptr<UnaryMeasurementXD<Eigen::Vector3d, 3>>& positionLandmarkMeasurementPtr,
                                       const std::string& worldFrameName, const std::string& imuFrameName,
                                       const Eigen::Isometry3d& T_I_sensorFrame, const int landmarkCreationCounter)
      : GmsfUnaryExpressionLandmark(positionLandmarkMeasurementPtr, worldFrameName, imuFrameName, T_I_sensorFrame, landmarkCreationCounter),
        positionLandmarkMeasurementPtr_(positionLandmarkMeasurementPtr) {}

  // Destructor
  ~GmsfUnaryExpressionLandmarkPosition3() = default;

  // Noise as GTSAM Datatype
  [[nodiscard]] const gtsam::Vector getNoiseDensity() const override {
    return positionLandmarkMeasurementPtr_->unaryMeasurementNoiseDensity();
  }

  // Return Measurement as GTSAM Datatype
  [[nodiscard]] const gtsam::Point3 getGtsamMeasurementValue() const override {
    return gtsam::Point3(positionLandmarkMeasurementPtr_->unaryMeasurement().matrix());
  }

 protected:

  // ii.B) Adding Landmark State in Dynamic Memory
  virtual gtsam::Point3 computeW_t_W_L_initial(const gtsam::NavState& W_currentPropagatedState) final {
    // Get initial guess (computed geometrically)
    const gtsam::Pose3& T_W_I_est = W_currentPropagatedState.pose();                                         // alias
    const gtsam::Point3& S_t_S_L_meas = gtsam::Point3(positionLandmarkMeasurementPtr_->unaryMeasurement());  // alias
    const gtsam::Pose3 T_W_S_est = T_W_I_est * gtsam::Pose3(T_I_sensorFrameInit_.matrix());
    const gtsam::Point3 W_t_W_S_est = T_W_S_est.translation();
    const gtsam::Point3 W_t_S_L_meas = T_W_S_est.rotation().rotate(S_t_S_L_meas);
    return W_t_W_S_est + W_t_S_L_meas;
  }

  // iii) Transform state to sensor frame
  void transformImuStateToSensorFrameState() final {
    // Gtsam Data Type
    const gtsam::Pose3 T_sensorFrame_I(T_I_sensorFrameInit_.inverse().matrix());

    // Transform to Sensor Frame
    exp_sensorFrame_t_sensorFrame_landmark_ =
        gtsam::transformFrom(gtsam::Pose3_(T_sensorFrame_I), exp_sensorFrame_t_sensorFrame_landmark_);  // S_t_S_L at this point
  }

  // iv) Extrinsic Calibration
  void applyExtrinsicCalibrationCorrection(const gtsam::Point3_& ) final {
    // TODO: Implement
    REGULAR_COUT << RED_START << "GmsfUnaryExpressionLandmarkPosition3: Extrinsic Calibration not implemented yet." << COLOR_END
                 << std::endl;
  }

  gtsam::Pose3 convertToPose3(const gtsam::Point3& measurement) final { return gtsam::Pose3(gtsam::Rot3::Identity(), measurement); }

  // Return Expression
  [[nodiscard]] const gtsam::Expression<gtsam::Point3> getGtsamExpression() const override {
    return exp_sensorFrame_t_sensorFrame_landmark_;
  }

 private:
  // Full Measurement Type
  std::shared_ptr<UnaryMeasurementXD<Eigen::Vector3d, 3>> positionLandmarkMeasurementPtr_;

};
}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_POSITION3_LANDMARK_H