/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef OPTIMIZER_BASE_HPP
#define OPTIMIZER_BASE_HPP

// GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>

// Workspace
#include "graph_msf/config/GraphConfig.h"

namespace graph_msf {

class OptimizerBase {
 public:
  // Constructior and Destructor
  explicit OptimizerBase(const std::shared_ptr<GraphConfig> graphConfigPtr) : graphConfigPtr_(graphConfigPtr) {}
  ~OptimizerBase() = default;

  // Virtual function to be implemented by derived classes
  // Add factors
  virtual bool update() = 0;
  virtual bool update(const gtsam::NonlinearFactorGraph& newGraphFactors, const gtsam::Values& newGraphValues,
                      const std::map<gtsam::Key, double>& newGraphKeysTimeStampMap, const int depth = 0) = 0;
  virtual bool updateExistingValues(const gtsam::Values& newGraphValues) = 0;
  // Run (Batch) Optimization and get result
  virtual void optimize(int maxIterations) = 0;
  virtual const gtsam::Values& getAllOptimizedStates() = 0;
  virtual gtsam::KeyVector getAllOptimizedKeys() = 0;
  virtual const gtsam::NonlinearFactorGraph& getNonlinearFactorGraph() const = 0;
  // Get keyTimestampMap
  virtual const std::map<gtsam::Key, double>& getFullKeyTimestampMap() = 0;

  // Calculate State at Key
  virtual gtsam::Pose3 calculateEstimatedPose3(const gtsam::Key& key) = 0;
  virtual gtsam::Vector3 calculateEstimatedVelocity3(const gtsam::Key& key) = 0;
  virtual gtsam::imuBias::ConstantBias calculateEstimatedBias(const gtsam::Key& key) = 0;
  virtual gtsam::Point3 calculateEstimatedPoint3(const gtsam::Key& key) = 0;
  virtual gtsam::Vector calculateEstimatedVector(const gtsam::Key& key) = 0;
  virtual double calculateEstimatedScalar(const gtsam::Key&) {
    throw std::logic_error("Scalar extraction is unsupported by this optimizer");
  }

  // Marginal Covariance
  virtual gtsam::Matrix calculateMarginalCovarianceMatrixAtKey(const gtsam::Key& key) = 0;
  // The block order MUST be [first_key, second_key], independently of Symbol
  // ordering. Implementations must use the optimizer's actual joint posterior.
  virtual gtsam::Matrix calculateJointCovarianceAtKeys(const gtsam::Key&, const gtsam::Key&) {
    throw std::logic_error("Joint covariance extraction is unsupported by this optimizer");
  }

  // Coupled articulated states require all-or-nothing updates; filtering failed
  // factors or inserting recovery priors would silently change their model.
  void setRejectRecovery(bool value) { rejectRecovery_ = value; }

  // Add latest IMU bias before optimization
  virtual void addLatestImuBiasBelief(const gtsam::imuBias::ConstantBias& imuBias) { latestImuBias_ = imuBias; }

  // Add latest pose before optimization
  virtual void addLatestPoseBelief(const gtsam::Pose3& pose) { latestPose_ = pose; }

 protected:
  bool rejectRecovery_{false};
  // Config
  std::shared_ptr<GraphConfig> graphConfigPtr_;

  // Latest IMU bias
  gtsam::imuBias::ConstantBias latestImuBias_;
  gtsam::Pose3 latestPose_;
};

}  // namespace graph_msf

#endif  // OPTIMIZER_BASE_HPP
