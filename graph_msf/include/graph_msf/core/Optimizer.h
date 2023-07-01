/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef Optimizer_HPP
#define Optimizer_HPP

// GTSAM
#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>

// Workspace
#include "graph_msf/config/GraphConfig.h"

namespace graph_msf {

class Optimizer {
 public:
  // Constructior and Destructor
  explicit Optimizer(const std::shared_ptr<GraphConfig> graphConfigPtr) : graphConfigPtr_(graphConfigPtr) {}
  ~Optimizer() = default;

  // Virtual function to be implemented by derived classes
  virtual bool update() = 0;
  virtual bool update(const gtsam::NonlinearFactorGraph& newGraphFactors, const gtsam::Values& newGraphValues,
                      const std::map<gtsam::Key, double>& newGraphKeysTimeStampMap) = 0;

  // Calculate State at Key
  virtual gtsam::Pose3 calculateEstimatedPose(const gtsam::Key& key) = 0;
  virtual gtsam::Vector3 calculateEstimatedVelocity(const gtsam::Key& key) = 0;
  virtual gtsam::imuBias::ConstantBias calculateEstimatedBias(const gtsam::Key& key) = 0;

  // Marginal Covariance
  virtual gtsam::Matrix marginalCovariance(const gtsam::Key& key) = 0;

 protected:
  // Optimizer itself
  std::shared_ptr<gtsam::FixedLagSmoother> fixedLagSmootherPtr_;
  // Config
  std::shared_ptr<GraphConfig> graphConfigPtr_;
};

}  // namespace graph_msf

#endif  // Optimizer_HPP
