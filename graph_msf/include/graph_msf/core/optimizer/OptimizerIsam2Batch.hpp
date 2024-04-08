/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef OPTIMIZER_ISAM2_BATCH_HPP
#define OPTIMIZER_ISAM2_BATCH_HPP

// Workspace
#include <graph_msf/core/optimizer/OptimizerIsam2.hpp>

namespace graph_msf {

class OptimizerIsam2Batch : public OptimizerIsam2 {
 public:
  explicit OptimizerIsam2Batch(const std::shared_ptr<GraphConfig> graphConfigPtr) : OptimizerIsam2(graphConfigPtr) {
    // Initialize Slow Bundle Adjustement Smoother (if desired) -----------------------------------------------
    if (graphConfigPtr_->useAdditionalSlowBatchSmoother_) {
      // Initialize Slow Bundle Adjustement Smoother
      batchSmootherPtr_ = std::make_shared<gtsam::ISAM2>(isam2Params_);
      batchSmootherPtr_->params().print("GraphMSF: Factor Graph Parameters of Slow Batch Optimization Graph.");
    }
  }
  ~OptimizerIsam2Batch() = default;

  // Empty update call --> do nothing
  bool update() override { return true; }

  // Add to graph without running optimization
  bool update(const gtsam::NonlinearFactorGraph& newGraphFactors, const gtsam::Values& newGraphValues,
              const std::map<gtsam::Key, double>& newGraphKeysTimeStampMap) override {
    // Add Bundle Adjustement Smoother factors to batch without running optimization
    containerBatchSmootherFactors_.add(newGraphFactors);
    containerBatchSmootherValues_.insert(newGraphValues);
    // Add to keyTimestampMap
    batchSmootherKeyTimestampMap_.insert(newGraphKeysTimeStampMap.begin(), newGraphKeysTimeStampMap.end());
    return true;
  }

  // Optimize
  void optimize(int maxIterations) override {
    // Print
    std::cout << YELLOW_START << "GraphMSF: OptimizerIsam2Batch" << GREEN_START << " Optimizing slow batch smoother." << COLOR_END
              << std::endl;

    // Optimize
    batchSmootherIsam2Result_ = batchSmootherPtr_->update(containerBatchSmootherFactors_, containerBatchSmootherValues_);
    // Reset containers
    containerBatchSmootherFactors_.resize(0);
    containerBatchSmootherValues_.clear();

    // Get optimized result
    batchSmootherOptimizedResult_ = batchSmootherPtr_->calculateEstimate();

    // Flag
    optimizedAtLeastOnceFlag_ = true;
  }

  // Optimize bundle adjustement smoother (if desired)
  const gtsam::Values& getAllOptimizedStates() override {
    // Check
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerIsam2Batch: No optimization has been performed yet.");
    }

    // Return result
    return batchSmootherOptimizedResult_;
  }

  // Get all keys of optimized states
  gtsam::KeyVector getAllOptimizedKeys() override { return batchSmootherOptimizedResult_.keys(); }

  // Get nonlinear factor graph
  const gtsam::NonlinearFactorGraph& getNonlinearFactorGraph() const override { return batchSmootherPtr_->getFactorsUnsafe(); }

  // Get keyTimestampMap
  const std::map<gtsam::Key, double>& getFullKeyTimestampMap() override { return batchSmootherKeyTimestampMap_; }

  // Calculate States
  gtsam::Pose3 calculateEstimatedPose(const gtsam::Key& key) override { return batchSmootherPtr_->calculateEstimate<gtsam::Pose3>(key); }

  gtsam::Vector3 calculateEstimatedVelocity(const gtsam::Key& key) override {
    return batchSmootherPtr_->calculateEstimate<gtsam::Vector3>(key);
  }

  gtsam::imuBias::ConstantBias calculateEstimatedBias(const gtsam::Key& key) override {
    return batchSmootherPtr_->calculateEstimate<gtsam::imuBias::ConstantBias>(key);
  }

  gtsam::Point3 calculateEstimatedDisplacement(const gtsam::Key& key) override {
    return batchSmootherPtr_->calculateEstimate<gtsam::Point3>(key);
  }

  gtsam::Vector calculateStateAtKey(const gtsam::Key& key) override { return batchSmootherPtr_->calculateEstimate<gtsam::Vector>(key); }

  // Marginal Covariance
  gtsam::Matrix marginalCovariance(const gtsam::Key& key) override { return batchSmootherPtr_->marginalCovariance(key); }

 private:
  // Optimizer itself
  std::shared_ptr<gtsam::ISAM2> batchSmootherPtr_;
  // Containers for bundle adjustment smoother
  gtsam::NonlinearFactorGraph containerBatchSmootherFactors_;
  gtsam::Values containerBatchSmootherValues_;
  // Result and keyTimestampMap
  gtsam::ISAM2Result batchSmootherIsam2Result_;
  gtsam::Values batchSmootherOptimizedResult_;
  std::map<gtsam::Key, double> batchSmootherKeyTimestampMap_;
};

}  // namespace graph_msf

#endif  // OPTIMIZER_ISAM2_BATCH_HPP
