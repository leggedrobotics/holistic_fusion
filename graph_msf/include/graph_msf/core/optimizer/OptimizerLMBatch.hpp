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

    // Set Sorted Keys by Time Flag
    dividedIntoSubGraphsFlag_ = false;
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
  [[nodiscard]] const gtsam::NonlinearFactorGraph& getNonlinearFactorGraph() const override { return batchSmootherPtr_->graph(); }

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

  static std::vector<double> linspace(double start, double end, int num) {
    std::vector<double> result;
    if (num <= 0) return result;  // Empty vector if num is 0 or negative
    if (num == 1) {               // Single point case
      result.push_back(start);
      return result;
    }

    double step = (end - start) / (num - 1);
    for (int i = 0; i < num; ++i) {
      result.push_back(start + i * step);
    }

    return result;
  }

  void divideGraphIntoSubGraphs() {
    // Check
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: computeAndStoreAllMarginals: No optimization has been performed yet.");
    }

    // All Keys
    gtsam::KeyVector keysSorted = valuesLastOptimizedResult_.keys();

    // Sort keys by time (given in keyTimestampMap)
    std::sort(keysSorted.begin(), keysSorted.end(), [&](const gtsam::Key& key1, const gtsam::Key& key2) {
      return batchSmootherKeyTimestampMap_.at(key1) < batchSmootherKeyTimestampMap_.at(key2);
    });

    // Get time span and number of keys
    double startTime = batchSmootherKeyTimestampMap_.at(keysSorted.front());
    double endTime = batchSmootherKeyTimestampMap_.at(keysSorted.back());
    int numKeys = keysSorted.size();
    double timeDifference = endTime - startTime;

    // Print
    std::cout << "GraphMSF: OptimizerLMBatch: Graph consisting of " << numKeys << " keys is spanning " << timeDifference << " seconds."
              << std::endl;

    // Determine how many overlapping sub-graphs of size windowSizeSecond to create
    const double& windowSizeSeconds = graphConfigPtr_->windowSizeSecondsForMarginalsComputation_;  // Alias
    int numSubGraphs = std::ceil(timeDifference / windowSizeSeconds);
    std::cout << "GraphMSF: OptimizerLMBatch: Dividing graph into " << numSubGraphs << " main windows." << std::endl;
    numSubGraphs += (numSubGraphs - 1);  // Overlapping windows
    std::cout << "GraphMSF: OptimizerLMBatch: Dividing graph into " << numSubGraphs
              << " overlapping windows (to make sure there are no edges)." << std::endl;

    // Placeholders
    std::vector<double> subGraphCenterTimes;
    // Sub-graph start and end times
    std::vector<double> subGraphStartTimes;
    std::vector<double> subGraphEndTimes;

    // More than one sub-graph
    if (numSubGraphs > 1) {
      // Sub-graph Center Times: linearly spaced from startTime + windowSizeSeconds/2 to endTime - windowSizeSeconds/2
      subGraphCenterTimes = linspace(startTime + windowSizeSeconds / 2, endTime - windowSizeSeconds / 2, numSubGraphs);
      for (int i = 0; i < numSubGraphs; ++i) {
        subGraphStartTimes.push_back(subGraphCenterTimes[i] - windowSizeSeconds / 2);
        subGraphEndTimes.push_back(subGraphCenterTimes[i] + windowSizeSeconds / 2);
      }
      // One sub-graph
    } else {
      // Only one sub-graph
      subGraphCenterTimes.push_back((startTime + endTime) / 2);
      subGraphStartTimes.push_back(startTime);
      subGraphEndTimes.push_back(endTime);
    }

    // Print Relative Times
    for (int i = 0; i < numSubGraphs; ++i) {
      std::cout << "Subgraph " << i << " starts at relative " << subGraphStartTimes[i] - startTime << " seconds and ends at relative "
                << subGraphEndTimes[i] - startTime << " seconds." << std::endl;
      std::cout << "Center time: " << subGraphCenterTimes[i] - startTime << " seconds." << std::endl;
    }

    // Reset all containers
    subGraphs_.clear();
    keyToSubGraphIndexMap_.clear();
    marginalsForSubGraphs_.clear();

    // Create sub-graphs
    subGraphs_.resize(numSubGraphs);
    keyToSubGraphIndexMap_.clear();
    marginalsForSubGraphs_.resize(numSubGraphs);

    // Go through all factors and add them to the sub-graph if they are within the time window
    for (const auto& factorPtr : graphLastOptimizedResult_) {
      bool factorAtLeastInOneSubGraph = false;
      // Check whether the factor contains any key within the window of all sub-graphs
      for (int i = 0; i < numSubGraphs; ++i) {
        // Check whether factor contains key within window
        bool factorContainsKeyWithinSubGraphWindow = false;
        // Go through all keys of factor
        for (const auto& factorKey : factorPtr->keys()) {
          if (batchSmootherKeyTimestampMap_[factorKey] >= subGraphStartTimes[i] &&
              batchSmootherKeyTimestampMap_[factorKey] <= subGraphEndTimes[i]) {
            factorContainsKeyWithinSubGraphWindow = true;
            break;
          }
        }
        // Add factor if it contains key within window
        if (factorContainsKeyWithinSubGraphWindow) {
          subGraphs_[i].add(factorPtr);
          factorAtLeastInOneSubGraph = true;
        }
      }
      // Has to be in at least one sub-graph
      if (!factorAtLeastInOneSubGraph) {
        throw std::runtime_error("GraphMSF: OptimizerLMBatch: Factor does not contain any key within the time window of any sub-graph.");
      }
    }

    // Assign each key to the sub-graph with the closest center time to know in which sub-graph to compute marginals
    for (const auto& key : keysSorted) {
      double keyTime = batchSmootherKeyTimestampMap_[key];
      // Find the closest center time
      double minTimeDifference = std::numeric_limits<double>::max();
      int closestSubGraphIndex = -1;
      // Go through all sub-graphs
      for (int i = 0; i < numSubGraphs; ++i) {
        double timeDifference = std::abs(subGraphCenterTimes[i] - keyTime);
        if (timeDifference < minTimeDifference) {
          minTimeDifference = timeDifference;
          closestSubGraphIndex = i;
        }
      }
      // Assign key to sub-graph
      keyToSubGraphIndexMap_[key] = closestSubGraphIndex;
    }

    // Optimize each sub-graph and compute marginals
    for (int i = 0; i < numSubGraphs; ++i) {
      // Optimize
      marginalsForSubGraphs_[i] = gtsam::Marginals(subGraphs_[i], valuesLastOptimizedResult_);
      // Print
      std::cout << "GraphMSF: OptimizerLMBatch: Subgraph " << i << " optimized and marginals computed." << std::endl;
    }

    // Set flag
    dividedIntoSubGraphsFlag_ = true;

    // Print
    std::cout << "GraphMSF: OptimizerLMBatch: Divided graph into subgraphs." << std::endl;
  }

  // Marginal Covariance
  gtsam::Matrix calculateMarginalCovarianceMatrixAtKey(const gtsam::Key& valueKey) override {
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: OptimizerLMBatch: marginalCovariance: No optimization has been performed yet.");
    }

    if (!dividedIntoSubGraphsFlag_) {
      divideGraphIntoSubGraphs();
    }

    // Super slow check
    if constexpr (false) {
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
    }

    // Only use window around key for marginal covariance ------------------------------------------------
    if (graphConfigPtr_->useWindowForMarginalsComputationFlag_) {
      // Get sub-graph index
      const int& subGraphsIndex_ = keyToSubGraphIndexMap_[valueKey];  // Alias
      if (subGraphsIndex_ != currentOptimizedSubgraphIndex_) {
        std::cout << GREEN_START << "Switching to sub-graph " << subGraphsIndex_ << " for marginal covariance computation." << COLOR_END
                  << std::endl;
        currentOptimizedSubgraphIndex_ = subGraphsIndex_;
      }
      // Compute marginal covariance and measure time
      gtsam::Matrix marginalCovariance = marginalsForSubGraphs_[subGraphsIndex_].marginalCovariance(valueKey);
      return marginalCovariance;
    }
    // Normal case: Compute marginals for whole graph
    else {
      // Have to compute all marginals (if not done already for this result)
      if (!marginalsComputedForLastOptimizedResultFlag_) {
        std::cout << "Getting new marginals for last optimized result." << std::endl;
        marginalsForLastOptimizedResult_ = gtsam::Marginals(graphLastOptimizedResult_, valuesLastOptimizedResult_);
        marginalsComputedForLastOptimizedResultFlag_ = true;
      }
      // Return
      return marginalsForLastOptimizedResult_.marginalCovariance(valueKey);
    }
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
  // Sub-graphs for marginal covariance computation
  bool dividedIntoSubGraphsFlag_ = false;
  std::vector<gtsam::NonlinearFactorGraph> subGraphs_;
  std::unordered_map<gtsam::Key, int> keyToSubGraphIndexMap_;
  std::vector<gtsam::Marginals> marginalsForSubGraphs_;
  // Container for ingredients of last optimization, e.g. for marginal covariance
  gtsam::NonlinearFactorGraph graphLastOptimizedResult_;
  bool marginalsComputedForLastOptimizedResultFlag_ = false;
  gtsam::Marginals marginalsForLastOptimizedResult_;

  // Indicator of current sub-graph
  int currentOptimizedSubgraphIndex_ = -1;
};

}  // namespace graph_msf

#endif  // OPTIMIZER_LM_BATCH_HPP
