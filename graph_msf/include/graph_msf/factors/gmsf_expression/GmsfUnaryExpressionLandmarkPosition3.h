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

// Workspace
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionLandmark.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"

namespace graph_msf {

class GmsfUnaryExpressionLandmarkPosition3 final : public GmsfUnaryExpressionLandmark<gtsam::Point3> {
 public:
  // Constructor
  GmsfUnaryExpressionLandmarkPosition3(const std::shared_ptr<UnaryMeasurementXD<Eigen::Vector3d, 3>>& positionLandmarkMeasurementPtr,
                                       const std::string& worldFrameName, const Eigen::Isometry3d& T_I_sensorFrame)
      : GmsfUnaryExpressionLandmark(positionLandmarkMeasurementPtr, worldFrameName, T_I_sensorFrame),
        positionLandmarkMeasurementPtr_(positionLandmarkMeasurementPtr),
        exp_sensorFrame_t_sensorFrame_landmark_(gtsam::Point3::Identity()),
        exp_T_W_I_(gtsam::Pose3::Identity()),
        landmarkName_(positionLandmarkMeasurementPtr->measurementName()) {}

  // Destructor
  ~GmsfUnaryExpressionLandmarkPosition3() override = default;

  // i) Generate Expression for Basic IMU State in World Frame at Key
  void generateExpressionForBasicImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) override {
    // Get robot state
    exp_T_W_I_ = gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKey));
  }

  // ii.B) Adding Landmark State in Dynamic Memory
  void convertRobotAndLandmarkStatesToMeasurement(TransformsExpressionKeys& transformsExpressionKeys) override {}

  // iii) Transform state to sensor frame
  void transformStateToSensorFrame() override {}

  // iv) Extrinsic Calibration
  void addExtrinsicCalibrationCorrection(TransformsExpressionKeys& transformsExpressionKeys) override {
    // TODO: Implement
    REGULAR_COUT << RED_START << "GmsfUnaryExpressionLandmarkPosition3: Extrinsic Calibration not implemented yet." << COLOR_END
                 << std::endl;
  }

  // Accessors
  [[nodiscard]] const auto& getUnaryMeasurementPtr() const { return positionLandmarkMeasurementPtr_; }

  // Noise as GTSAM Datatype
  [[nodiscard]] const gtsam::Vector getNoiseDensity() const override {
    return positionLandmarkMeasurementPtr_->unaryMeasurementNoiseDensity();
  }

  // Return Measurement as GTSAM Datatype
  [[nodiscard]] const gtsam::Point3 getMeasurement() const override {
    return gtsam::Point3(positionLandmarkMeasurementPtr_->unaryMeasurement().matrix());
  }

  // Return Expression
  [[nodiscard]] const gtsam::Expression<gtsam::Point3> getExpression() const override { return exp_sensorFrame_t_sensorFrame_landmark_; }

 private:
  // Full Measurement Type
  std::shared_ptr<UnaryMeasurementXD<Eigen::Vector3d, 3>> positionLandmarkMeasurementPtr_;

  // Landmark Identifier
  const std::string& landmarkName_;

  // Expression
  gtsam::Point3_ exp_sensorFrame_t_sensorFrame_landmark_;  // Measurement --> this is what h(x) has to be
  gtsam::Pose3_ exp_T_W_I_;                                // Robot Pose
};
}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_POSITION3_LANDMARK_H