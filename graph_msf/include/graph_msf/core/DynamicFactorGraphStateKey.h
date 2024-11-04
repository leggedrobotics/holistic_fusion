/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Workspace
#include "graph_msf/core/DynamicVariableType.h"
#include "graph_msf/interface/Terminal.h"

#ifndef FACTOR_GRAPH_STATE_KEY_H
#define FACTOR_GRAPH_STATE_KEY_H

// Output
#define REGULAR_COUT std::cout << YELLOW_START << "GMSF-FactorGraphStateKey" << COLOR_END

namespace graph_msf {

template <class GTSAM_TRANSFORM_TYPE>  // e.g. gtsam::Pose3, gtsam::Point3
class DynamicFactorGraphStateKey {
 public:
  // Constructor
  DynamicFactorGraphStateKey(const gtsam::Key& key, const double time, const int numberStepsOptimized,
                             const GTSAM_TRANSFORM_TYPE& approximateTransformationBeforeOptimization, DynamicVariableType variableType,
                             const std::string& frame1, const std::string& frame2)
      : key_(key),
        time_(time),
        numberStepsOptimized_(numberStepsOptimized),
        approximateTransformationBeforeOptimization_(approximateTransformationBeforeOptimization),
        variableType_(std::move(variableType)),
        isVariableActive_(true),
        frame1_(frame1),
        frame2_(frame2) {}

  // Default constructor for creating identity object
  DynamicFactorGraphStateKey() = default;

  // Destructor
  ~DynamicFactorGraphStateKey() = default;

  // Accessors
  const gtsam::Key key() const { return key_; }
  int getNumberStepsOptimized() const { return numberStepsOptimized_; }
  double getTime() const { return time_; }
  [[nodiscard]] GTSAM_TRANSFORM_TYPE getApproximateTransformationBeforeOptimization() const {
    return approximateTransformationBeforeOptimization_;
  }
  [[nodiscard]] Eigen::Vector3d getReferenceFrameKeyframePosition() const { return variableType_.getReferenceFrameKeyframePosition(); }
  [[nodiscard]] double getVariableCreationTime() const { return variableType_.getVariableCreationTime(); }
  double computeVariableAge(const double currentTime) const { return currentTime - variableType_.getVariableCreationTime(); }
  [[nodiscard]] const DynamicVariableTypeEnum& getVariableTypeEnum() const { return variableType_.getVariableTypeEnum(); }
  [[nodiscard]] bool isVariableActive() const { return isVariableActive_; }
  [[nodiscard]] const GTSAM_TRANSFORM_TYPE& getTransformationAfterOptimization() const { return transformationAfterOptimization_; }
  [[nodiscard]] const gtsam::Matrix66& getCovarianceAfterOptimization() const { return covarianceAfterOptimization_; }

  // Setters
  /// Members
  void setTimeStamp(const double time) { time_ = time; }
  void resetNumberStepsOptimized() { numberStepsOptimized_ = 0; }

  /// Status
  void activateVariable() {
    isVariableActive_ = true;
    REGULAR_COUT << " Activated Variable at Key " << gtsam::Symbol(key_) << std::endl;
  }
  void deactivateVariable() {
    isVariableActive_ = false;
    REGULAR_COUT << " Deactivated Variable at Key " << gtsam::Symbol(key_) << " between " << frame1_ << " and " << frame2_ << std::endl;
  }

  /// Optimization
  void incrementNumberStepsOptimized() { ++numberStepsOptimized_; }

  void setApproximateTransformationBeforeOptimization(const gtsam::Pose3& approximateTransformationBeforeOptimization) {
    approximateTransformationBeforeOptimization_ = approximateTransformationBeforeOptimization;
  }

  void updateLatestEstimate(const GTSAM_TRANSFORM_TYPE& transformationAfterOptimization,
                            const gtsam::Matrix66& covarianceAfterOptimization) {
    transformationAfterOptimization_ = transformationAfterOptimization;
    covarianceAfterOptimization_ = covarianceAfterOptimization;
  }

 private:
  // Members
  gtsam::Key key_ = -1;
  double time_ = 0.0;

  // Variable Type and status
  DynamicVariableType variableType_ = DynamicVariableType::Global(0.0);
  bool isVariableActive_ = true;  // Always active in the beginning when added

  // Optimization
  int numberStepsOptimized_ = 0;
  GTSAM_TRANSFORM_TYPE approximateTransformationBeforeOptimization_;
  GTSAM_TRANSFORM_TYPE transformationAfterOptimization_;
  gtsam::Matrix66 covarianceAfterOptimization_;

  // Frames
  std::string frame1_ = "";
  std::string frame2_ = "";
};

}  // namespace graph_msf

#endif  // FACTOR_GRAPH_STATE_KEY_H
