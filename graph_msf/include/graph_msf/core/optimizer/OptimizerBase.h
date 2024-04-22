/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef OPTIMIZER_BASE_HPP
#define OPTIMIZER_BASE_HPP

// GTSAM
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
                      const std::map<gtsam::Key, double>& newGraphKeysTimeStampMap) = 0;
  // Run (Batch) Optimization and get result
  virtual void optimize(int maxIterations) = 0;
  virtual const gtsam::Values& getAllOptimizedStates() = 0;
  virtual gtsam::KeyVector getAllOptimizedKeys() = 0;
  virtual const gtsam::NonlinearFactorGraph& getNonlinearFactorGraph() const = 0;
  // Get keyTimestampMap
  virtual const std::map<gtsam::Key, double>& getFullKeyTimestampMap() = 0;

  // Calculate State at Key
  virtual gtsam::Pose3 calculateEstimatedPose(const gtsam::Key& key) = 0;
  virtual gtsam::Vector3 calculateEstimatedVelocity(const gtsam::Key& key) = 0;
  virtual gtsam::imuBias::ConstantBias calculateEstimatedBias(const gtsam::Key& key) = 0;
  virtual gtsam::Point3 calculateEstimatedDisplacement(const gtsam::Key& key) = 0;
  virtual gtsam::Vector calculateStateAtKey(const gtsam::Key& key) = 0;

  // Marginal Covariance
  virtual gtsam::Matrix marginalCovariance(const gtsam::Key& key) = 0;

 protected:
  // Config
  std::shared_ptr<GraphConfig> graphConfigPtr_;
};

}  // namespace graph_msf

#endif  // OPTIMIZER_BASE_HPP
