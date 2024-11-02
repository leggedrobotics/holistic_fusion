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
    if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
      REGULAR_COUT << YELLOW_START << "GraphMSF: OptimizerLMBatch" << GREEN_START
                   << " Initializing slow batch smoother that is optimized with LM." << COLOR_END << std::endl;
      // Set whether to use Cholesky factorization
      if (graphConfigPtr_->slowBatchSmootherUseCholeskyFactorizationFlag_) {
        lmParams_.linearSolverType = gtsam::NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY;
        REGULAR_COUT << "Using Multifrontal-Cholesky factorization for slow batch smoother (LM)." << std::endl;
      } else {
        lmParams_.linearSolverType = gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR;
        REGULAR_COUT << "Using Multifrontal-QR factorization for slow batch smoother (LM)." << std::endl;
      }
      // Print
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

    // Optimize
    valuesLastOptimizedResult_ = batchSmootherPtr_->optimize();
    marginalsComputedForLastOptimizedResultFlag_ = false;
    optimizedAtLeastOnceFlag_ = true;
  }

  // Optimize bundle adjustement smoother (if desired)
  const gtsam::Values& getAllOptimizedStates() override {
    // Check
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: getAllOptimizedStates: No optimization has been performed yet.");
    }

    // Return result
    return valuesLastOptimizedResult_;
  }

  // Get all keys of optimized states
  gtsam::KeyVector getAllOptimizedKeys() override { return valuesLastOptimizedResult_.keys(); }

  // Get nonlinear factor graph
  const gtsam::NonlinearFactorGraph& getNonlinearFactorGraph() const override { return batchSmootherPtr_->graph(); }

  // Get keyTimestampMap
  const std::map<gtsam::Key, double>& getFullKeyTimestampMap() override { return batchSmootherKeyTimestampMap_; }

  // Calculate State / Covariance --------------------------------------------------------------
  // Pose3
  gtsam::Pose3 calculateEstimatedPose3(const gtsam::Key& key) override {
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: calculateEstimatedPose: No optimization has been performed yet.");
    }
    return valuesLastOptimizedResult_.at<gtsam::Pose3>(key);
  }
  // Velocity3
  gtsam::Vector3 calculateEstimatedVelocity3(const gtsam::Key& key) override {
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: calculateEstimatedVelocity: No optimization has been performed yet.");
    }
    return valuesLastOptimizedResult_.at<gtsam::Vector3>(key);
  }
  // Bias
  gtsam::imuBias::ConstantBias calculateEstimatedBias(const gtsam::Key& key) override {
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: calculateEstimatedBias: No optimization has been performed yet.");
    }
    return valuesLastOptimizedResult_.at<gtsam::imuBias::ConstantBias>(key);
  }
  // Point3
  gtsam::Point3 calculateEstimatedPoint3(const gtsam::Key& key) override {
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: calculateEstimatedDisplacement: No optimization has been performed yet.");
    }
    return valuesLastOptimizedResult_.at<gtsam::Point3>(key);
  }
  // Vector
  gtsam::Vector calculateEstimatedVector(const gtsam::Key& key) override {
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: calculateStateAtKey: No optimization has been performed yet.");
    }
    return valuesLastOptimizedResult_.at<gtsam::Vector>(key);
  }

  // Marginal Covariance
  gtsam::Matrix calculateMarginalCovarianceMatrixAtKey(const gtsam::Key& valueKey) override {
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: marginalCovariance: No optimization has been performed yet.");
    }

    // Only use window around key for marginal covariance ------------------------------------------------
    if (graphConfigPtr_->useWindowForMarginalsComputationFlag_) {
      // Get window size
      const double& windowSize = graphConfigPtr_->windowSizeForMarginalsComputation_;  // Alias
      // Get key timestamp
      const double keyTimestamp = batchSmootherKeyTimestampMap_.at(valueKey);

      // Get subgraph
      gtsam::NonlinearFactorGraph subGraph;
      // Check for EVERY factor whether it is close enough to current key --> total squared complexity (check O(N) factors for O(N) keys)
        for (const auto& factorPtr : graphLastOptimizedResult_) {
          bool factorContainsKeyWithinWindow = false;
          // Check whether the factor contains any key within the window
          for (const auto& factorKey : factorPtr->keys()) {
            if (std::abs(batchSmootherKeyTimestampMap_[factorKey] - keyTimestamp) < windowSize) {
                factorContainsKeyWithinWindow = true;
                break;
            }
          }
          // Add factor if it contains key within window
          if (factorContainsKeyWithinWindow) {
            subGraph.add(factorPtr);
          }
        }

      // Get marginals
      marginalsForLastOptimizedResult_ = gtsam::Marginals(subGraph, valuesLastOptimizedResult_);
    }
    // Normal case: Compute marginals for whole graph
    else {
      // Have to compute all marginals (if not done already for this result)
      if (!marginalsComputedForLastOptimizedResultFlag_) {
        std::cout << "Getting new marginals for last optimized result." << std::endl;
        marginalsForLastOptimizedResult_ = gtsam::Marginals(graphLastOptimizedResult_, valuesLastOptimizedResult_);
        marginalsComputedForLastOptimizedResultFlag_ = true;
      }
    }

    // Print size of marginals
    std::cout << "Key: " << gtsam::Symbol(valueKey) << std::endl;
    std::cout << "Size of marginals: " << marginalsForLastOptimizedResult_.optimize().size() << std::endl;

    // Check whether key exists in optimized result
    if (!valuesLastOptimizedResult_.exists(valueKey)) {
      std::cout << "Key " << gtsam::Symbol(valueKey) << " does not exist in optimized result." << std::endl;
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: marginalCovariance: Key does not exist in optimized result.");
    }
    // Check whether key exists in nonlinar factor graph
    if (!graphLastOptimizedResult_.keys().exists(valueKey)) {
      std::cout << "Key " << gtsam::Symbol(valueKey) << " does not exist in nonlinear factor graph." << std::endl;
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: marginalCovariance: Key does not exist in nonlinear factor graph.");
    }

    // Return
    return marginalsForLastOptimizedResult_.marginalCovariance(valueKey);
  }

 private:
  // Optimizer itself
  std::shared_ptr<gtsam::LevenbergMarquardtOptimizer> batchSmootherPtr_;
  // Containers for bundle adjustment smoother
  gtsam::NonlinearFactorGraph containerBatchSmootherFactors_;
  gtsam::Values containerBatchSmootherValues_;
  // Result and keyTimestampMap
  bool optimizedAtLeastOnceFlag_ = false;
  gtsam::Values valuesLastOptimizedResult_;
  std::map<gtsam::Key, double> batchSmootherKeyTimestampMap_;
  // Container for ingredients of last optimization, e.g. for marginal covariance
  gtsam::NonlinearFactorGraph graphLastOptimizedResult_;
  bool marginalsComputedForLastOptimizedResultFlag_ = false;
  gtsam::Marginals marginalsForLastOptimizedResult_;
};

}  // namespace graph_msf

#endif  // OPTIMIZER_LM_BATCH_HPP
