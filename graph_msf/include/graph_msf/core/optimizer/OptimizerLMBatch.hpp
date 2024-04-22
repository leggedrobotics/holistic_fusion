/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef OPTIMIZER_LM_BATCH_HPP
#define OPTIMIZER_LM_BATCH_HPP

// GTSAM
#include <gtsam/nonlinear/Marginals.h>

// Workspace
#include <graph_msf/core/optimizer/OptimizerLM.hpp>

namespace graph_msf {

class OptimizerLMBatch : public OptimizerLM {
 public:
  explicit OptimizerLMBatch(const std::shared_ptr<GraphConfig> graphConfigPtr) : OptimizerLM(graphConfigPtr) {
    // Initialize Slow Bundle Adjustement Smoother (if desired) -----------------------------------------------
    if (graphConfigPtr_->useAdditionalSlowBatchSmoother_) {
      std::cout << YELLOW_START << "GraphMSF: OptimizerLMBatch" << GREEN_START
                << " Initializing slow batch smoother that is optimized with LM." << COLOR_END << std::endl;
      lmParams_.print("GraphMSF: OptimizerLMBatch, LM Parameters:");
    }
  }
  ~OptimizerLMBatch() = default;

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

    // Set LM Parameters
    lmParams_.maxIterations = maxIterations;

    // Initialize Slow Bundle Adjustment Smoother
    batchSmootherPtr_ =
        std::make_shared<gtsam::LevenbergMarquardtOptimizer>(containerBatchSmootherFactors_, containerBatchSmootherValues_, lmParams_);

    // Do not reset containers --> Will build up graph again from scratch for next optimization
    // containerBatchSmootherFactors_.resize(0);
    // containerBatchSmootherValues_.clear();
    // Log State of graph in order to compute marginal covariance if desired
    graphLastOptimizedResult_ = containerBatchSmootherFactors_;
    marginalsComputedForLastOptimizedResultFlag_ = false;

    // Optimize
    batchSmootherOptimizedResult_ = batchSmootherPtr_->optimize();
    optimizedAtLeastOnceFlag_ = true;
  }

  // Optimize bundle adjustement smoother (if desired)
  const gtsam::Values& getAllOptimizedStates() override {
    // Check
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: getAllOptimizedStates: No optimization has been performed yet.");
    }

    // Return result
    return batchSmootherOptimizedResult_;
  }

  // Get all keys of optimized states
  gtsam::KeyVector getAllOptimizedKeys() override { return batchSmootherOptimizedResult_.keys(); }

  // Get nonlinear factor graph
  const gtsam::NonlinearFactorGraph& getNonlinearFactorGraph() const override { return batchSmootherPtr_->graph(); }

  // Get keyTimestampMap
  const std::map<gtsam::Key, double>& getFullKeyTimestampMap() override { return batchSmootherKeyTimestampMap_; }

  // Calculate States
  gtsam::Pose3 calculateEstimatedPose(const gtsam::Key& key) override {
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: calculateEstimatedPose: No optimization has been performed yet.");
    }
    return batchSmootherOptimizedResult_.at<gtsam::Pose3>(key);
  }

  gtsam::Vector3 calculateEstimatedVelocity(const gtsam::Key& key) override {
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: calculateEstimatedVelocity: No optimization has been performed yet.");
    }
    return batchSmootherOptimizedResult_.at<gtsam::Vector3>(key);
  }

  gtsam::imuBias::ConstantBias calculateEstimatedBias(const gtsam::Key& key) override {
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: calculateEstimatedBias: No optimization has been performed yet.");
    }
    return batchSmootherOptimizedResult_.at<gtsam::imuBias::ConstantBias>(key);
  }

  gtsam::Point3 calculateEstimatedDisplacement(const gtsam::Key& key) override {
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: calculateEstimatedDisplacement: No optimization has been performed yet.");
    }
    return batchSmootherOptimizedResult_.at<gtsam::Point3>(key);
  }

  gtsam::Vector calculateStateAtKey(const gtsam::Key& key) override {
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: calculateStateAtKey: No optimization has been performed yet.");
    }
    return batchSmootherOptimizedResult_.at<gtsam::Vector>(key);
  }

  // Marginal Covariance
  gtsam::Matrix marginalCovariance(const gtsam::Key& key) override {
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: marginalCovariance: No optimization has been performed yet.");
    }

    // Have to compute all marginals (if not done already for this result
    if (!marginalsComputedForLastOptimizedResultFlag_) {
      marginalsForLastOptimizedResult_ = gtsam::Marginals(graphLastOptimizedResult_, batchSmootherOptimizedResult_);
    }

    // Return marginal covariance for key
    return marginalsForLastOptimizedResult_.marginalCovariance(key);
  }

 private:
  // Optimizer itself
  std::shared_ptr<gtsam::LevenbergMarquardtOptimizer> batchSmootherPtr_;
  // Containers for bundle adjustment smoother
  gtsam::NonlinearFactorGraph containerBatchSmootherFactors_;
  gtsam::Values containerBatchSmootherValues_;
  // Result and keyTimestampMap
  bool optimizedAtLeastOnceFlag_ = false;
  gtsam::Values batchSmootherOptimizedResult_;
  std::map<gtsam::Key, double> batchSmootherKeyTimestampMap_;
  // Container for ingredients of last optimization, e.g. for marginal covariance
  gtsam::NonlinearFactorGraph graphLastOptimizedResult_;
  bool marginalsComputedForLastOptimizedResultFlag_ = false;
  gtsam::Marginals marginalsForLastOptimizedResult_;
};

}  // namespace graph_msf

#endif  // OPTIMIZER_LM_BATCH_HPP
