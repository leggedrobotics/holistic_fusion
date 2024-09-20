/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef OPTIMIZER_ISAM2_FIXED_LAG_HPP
#define OPTIMIZER_ISAM2_FIXED_LAG_HPP

// GTSAM
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// Workspace
#include <graph_msf/core/optimizer/OptimizerIsam2.hpp>

namespace graph_msf {

class OptimizerIsam2FixedLag : public OptimizerIsam2 {
 public:
  explicit OptimizerIsam2FixedLag(const std::shared_ptr<GraphConfig> graphConfigPtr) : OptimizerIsam2(graphConfigPtr) {
    // Initialize Real-time Smoother -----------------------------------------------
    fixedLagSmootherPtr_ =
        std::make_shared<gtsam::IncrementalFixedLagSmoother>(graphConfigPtr_->realTimeSmootherLag_,
                                                             isam2Params_);  // std::make_shared<gtsam::ISAM2>(isamParams_);
    fixedLagSmootherPtr_->params().print("GraphMSF: Factor Graph Parameters of real-time graph.");
  }
  ~OptimizerIsam2FixedLag() = default;

  bool update() override {
    fixedLagSmootherPtr_->update();
    return true;
  }

  bool update(const gtsam::NonlinearFactorGraph& newGraphFactors, const gtsam::Values& newGraphValues,
              const std::map<gtsam::Key, double>& newGraphKeysTimeStampMap) override {
    // Make copy of the fixedLagSmootherPtr_ to avoid changing the original graph
    gtsam::IncrementalFixedLagSmoother fixedLagSmootherCopy = *fixedLagSmootherPtr_;

    // Try to update
    try {
      fixedLagSmootherPtr_->update(newGraphFactors, newGraphValues, newGraphKeysTimeStampMap);
    }
    // Case 1: Catching the failure of marginalization (as some states which should be marginalized have not been optimized before)
    catch (const std::out_of_range& outOfRangeException) {
      std::cerr << YELLOW_START << "GMsf-ISAM2" << RED_START << " Out of Range exception while optimizing graph: '"
                << outOfRangeException.what() << "'." << COLOR_END << std::endl;
      std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                << " This happens if the FixedLagSmoother tries to marginalize out states that have never been optimized before. This e.g. "
                   "happens if the graph hasn't been optimized for the duration of the smoother lag. Increase the lag or optimization "
                   "frequency in this case."
                << COLOR_END << std::endl;

      // Detect all factors that contain a timestamp older than the smoother lag
      double latestTimeStamp = 0.0;
      for (auto factor : newGraphFactors) {
        for (auto key : factor->keys()) {
          // Check whether key is in the keyTimestampMap
          if (newGraphKeysTimeStampMap.find(key) != newGraphKeysTimeStampMap.end()) {
            if (newGraphKeysTimeStampMap.at(key) > latestTimeStamp) {
              latestTimeStamp = newGraphKeysTimeStampMap.at(key);
            }
          }
        }
      }
      std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Latest timestamp in new factors: " << std::setprecision(14)
                << latestTimeStamp << COLOR_END << std::endl;
      // Filter out the factor that caused the error -------------------------
      bool filteredOutAtLeastOneKey = false;
      // Containers
      gtsam::NonlinearFactorGraph newGraphFactorsFiltered = gtsam::NonlinearFactorGraph();
      // Filtering the keyTimestampMap
      std::map<gtsam::Key, double> newGraphKeysTimeStampMapFiltered = newGraphKeysTimeStampMap;
      // For loop
      for (auto factor : newGraphFactors) {
        bool factorOnlyContainsExistentKeys = true;
        for (auto key : factor->keys()) {
          // If Exists
          if (newGraphKeysTimeStampMap.find(key) != newGraphKeysTimeStampMap.end()) {
            double timestampAtKey = newGraphKeysTimeStampMap.at(key);
            // Case 1: Check if key is older than smoother lag
            if (latestTimeStamp - timestampAtKey > graphConfigPtr_->realTimeSmootherLag_) {
              std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                        << " Factor contains key older than smoother lag: " << gtsam::Symbol(key) << ", which is "
                        << latestTimeStamp - timestampAtKey << "s old." << COLOR_END << std::endl;
              factorOnlyContainsExistentKeys = false;
              filteredOutAtLeastOneKey = true;
            }
            // Case 2: Check if key is from the future --> hence might not be optimized
            else if (timestampAtKey > latestTimeStamp) {
              std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Factor contains key from the future: " << gtsam::Symbol(key)
                        << ", which is " << timestampAtKey << "s in the future." << COLOR_END << std::endl;
              factorOnlyContainsExistentKeys = false;
              filteredOutAtLeastOneKey = true;
            }
          }
        }
        // Add factor if it does not contain any key older than the smoother lag
        if (factorOnlyContainsExistentKeys) {
          newGraphFactorsFiltered.add(factor);
        }
      }

      // Limit depth of recursion
      // Try again
      if (filteredOutAtLeastOneKey) {
        std::cout << YELLOW_START << "GMsf-ISAM2" << GREEN_START
                  << " Filtered out factors that are either older than the smoother lag or coming from the future. Trying to optimize "
                     "again."
                  << COLOR_END << std::endl;
        return update(newGraphFactorsFiltered, newGraphValues, newGraphKeysTimeStampMapFiltered);
      }
      // Nothing changed, hence abort
      else {
        // Show all values and corresponding timestamps that are not in timestamp map
        std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                  << " Could not filter out any factors that are either older than the smoother lag or coming from the future. Aborting "
                     "optimization."
                  << COLOR_END << std::endl;
        return false;
      }
    }
    // Case 2: Catching the addition of a bad factor (pointing to a state that does not exist)
    catch (const gtsam::ValuesKeyDoesNotExist& valuesKeyDoesNotExistException) {
      std::cout << "----------------------------------------------------------" << std::endl;
      std::cerr << YELLOW_START << "GMsf-ISAM2" << RED_START << " ValuesKeyDoesNotExist exception while optimizing graph: '" << COLOR_END
                << valuesKeyDoesNotExistException.what() << "'" << std::endl;
      std::cout << YELLOW_START << "GMsf-ISAM2" << COLOR_END
                << " This happens if a factor is added to the graph that contains a non-existent state. The most common case is a factor "
                   "containing a graph state that was marginalized out before. To avoid this, increase the factor graph lag or reduce "
                   "the delay of your measurements."
                << std::endl;

      // Filter out the factor that caused the error -------------------------
      gtsam::NonlinearFactorGraph newGraphFactorsFiltered;
      gtsam::Key valuesKeyNotExistent = valuesKeyDoesNotExistException.key();
      bool removedAtLeastOneFactorOrKey = false;
      for (auto factor : newGraphFactors) {
        bool factorContainsExistentKeys = true;
        for (auto key : factor->keys()) {
          if (key == valuesKeyNotExistent) {
            std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Factor contains non-existent key: " << gtsam::Symbol(key)
                      << ". -> Removed." << COLOR_END << std::endl;
            factorContainsExistentKeys = false;
            removedAtLeastOneFactorOrKey = true;
            break;  // factor contains dangerous key --> break inner loop
          }
        }
        if (factorContainsExistentKeys) {
          newGraphFactorsFiltered.add(factor);
        }
      }

      // Filter out the key from the keyTimestampMap -------------------------
      std::map<gtsam::Key, double> newGraphKeysTimeStampMapFiltered;
      for (auto keyTimeStamp : newGraphKeysTimeStampMap) {
        if (keyTimeStamp.first != valuesKeyNotExistent) {
          newGraphKeysTimeStampMapFiltered.insert(keyTimeStamp);
        } else {
          std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                    << " GraphKeysTimeStampMap contains non-existent key: " << gtsam::Symbol(keyTimeStamp.first) << ". -> Removed."
                    << COLOR_END << std::endl;
          removedAtLeastOneFactorOrKey = true;
        }
      }

      // Keys in values
      for (auto key : newGraphValues.keys()) {
        if (key == valuesKeyNotExistent) {
          std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Values contains non-existent key: " << gtsam::Symbol(key)
                    << ". -> Removed." << COLOR_END << std::endl;
          removedAtLeastOneFactorOrKey = true;
        }
      }

      // Potentially try to optimize again
      if (removedAtLeastOneFactorOrKey) {
        std::cout << YELLOW_START << "GMsf-ISAM2" << GREEN_START
                  << " Filtered out the factor or key that caused the error (i.e. containing the key "
                  << gtsam::Symbol(valuesKeyNotExistent) << "). Trying to optimize again." << COLOR_END << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
        // Copy back
        *fixedLagSmootherPtr_ = fixedLagSmootherCopy;
        // Try again
        return update(newGraphFactorsFiltered, newGraphValues, newGraphKeysTimeStampMapFiltered);
      } else {
        std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                  << " Could not filter out any factors that caused the error (i.e. containing the key "
                  << gtsam::Symbol(valuesKeyNotExistent) << "). Aborting optimization." << COLOR_END << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
        return false;
      }
    }
    // Case 3: ValuesKeyAlreadyExists
    catch (const gtsam::ValuesKeyAlreadyExists& valuesKeyAlreadyExistsException) {
      std::cerr << YELLOW_START << "GMsf-ISAM2" << RED_START << " ValuesKeyAlreadyExists exception while optimizing graph: '"
                << valuesKeyAlreadyExistsException.what() << "'" << COLOR_END << std::endl;
      std::cout << YELLOW_START << "GMsf-ISAM2" << COLOR_END
                << " This happens if a factor is added to the graph that contains a state that already exists. This is usually a sign of "
                   "a bug in the code. Please check the factor graph construction."
                << std::endl;
      throw std::runtime_error(valuesKeyAlreadyExistsException.what());
    }
    // Case 4: Runtime error --> can't handle explicitly
    catch (const std::runtime_error& runtimeError) {
      std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Runtime error while optimizing graph: " << runtimeError.what()
                << COLOR_END << std::endl;
      throw std::runtime_error(runtimeError.what());
    }
    if (!optimizedAtLeastOnceFlag_) {
      optimizedAtLeastOnceFlag_ = true;
    }
    return true;
  }

  // Optimize
  void optimize(int maxIterations) override {
    // Optimize
    fixedLagSmootherOptimizedResult_ = fixedLagSmootherPtr_->calculateEstimate();
    // Flag
    optimizedAtLeastOnceFlag_ = true;
  }

  // Get Result
  const gtsam::Values& getAllOptimizedStates() override {
    // Check
    if (!optimizedAtLeastOnceFlag_) {
      throw std::runtime_error("GraphMSF: Optimizer has not been optimized yet.");
    }

    // Return
    return fixedLagSmootherOptimizedResult_;
  }

  // Get all keys of optimized states
  gtsam::KeyVector getAllOptimizedKeys() override { return fixedLagSmootherOptimizedResult_.keys(); }

  // Get nonlinear factor graph
  const gtsam::NonlinearFactorGraph& getNonlinearFactorGraph() const { return fixedLagSmootherPtr_->getFactors(); }

  // Get keyTimestampMap
  const std::map<gtsam::Key, double>& getFullKeyTimestampMap() override { return fixedLagSmootherPtr_->timestamps(); }

  // Calculate State / Covariance --------------------------------------------------------------------------------------------
  // Pose3
  gtsam::Pose3 calculateEstimatedPose3(const gtsam::Key& key) override {
    return fixedLagSmootherPtr_->calculateEstimate<gtsam::Pose3>(key);
  }
  // Velocity3
  gtsam::Vector3 calculateEstimatedVelocity3(const gtsam::Key& key) override {
    return fixedLagSmootherPtr_->calculateEstimate<gtsam::Vector3>(key);
  }
  // Bias
  gtsam::imuBias::ConstantBias calculateEstimatedBias(const gtsam::Key& key) override {
    return fixedLagSmootherPtr_->calculateEstimate<gtsam::imuBias::ConstantBias>(key);
  }
  // Point3
  gtsam::Point3 calculateEstimatedPoint3(const gtsam::Key& key) override {
    return fixedLagSmootherPtr_->calculateEstimate<gtsam::Point3>(key);
  }
  // Vector
  gtsam::Vector calculateEstimatedVector(const gtsam::Key& key) override {
    return fixedLagSmootherPtr_->calculateEstimate<gtsam::Vector>(key);
  }

  // Marginal Covariance
  gtsam::Matrix calculateMarginalCovarianceMatrixAtKey(const gtsam::Key& key) override {
    // Check
    if (!optimizedAtLeastOnceFlag_) {
      REGULAR_COUT << RED_START << "GraphMSF: OptimizerIsam2FixedLag: No optimization has been performed yet." << COLOR_END << std::endl;
    }
    return fixedLagSmootherPtr_->marginalCovariance(key);
  }

 private:
  // Optimizer itself
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> fixedLagSmootherPtr_;
  // Result
  gtsam::Values fixedLagSmootherOptimizedResult_;
};

}  // namespace graph_msf

#endif  // OPTIMIZER_ISAM2_FIXED_LAG_HPP
