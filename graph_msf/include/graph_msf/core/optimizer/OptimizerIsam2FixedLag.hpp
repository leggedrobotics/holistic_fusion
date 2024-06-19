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
    // gtsam::IncrementalFixedLagSmoother fixedLagSmootherCopy = *fixedLagSmootherPtr_;

    // Try to update
    try {
      fixedLagSmootherPtr_->update(newGraphFactors, newGraphValues, newGraphKeysTimeStampMap);
    } catch (const std::out_of_range& outOfRangeException) {  // CASE 1 : Out of Range exception -----------------------------------
      std::cerr << YELLOW_START << "GMsf-ISAM2" << RED_START << " Out of Range exception while optimizing graph: '"
                << outOfRangeException.what() << "'." << COLOR_END << std::endl;
      std::cout
          << YELLOW_START << "GMsf-ISAM2" << RED_START
          << " This usually happens if the measurement delay is larger than the graph-smootherLag, i.e. the optimized graph instances are "
             "not connected. Increase the lag in this case."
          << COLOR_END << std::endl;

      // Overwrite with old fixed lag smoother
      //*fixedLagSmootherPtr_ = fixedLagSmootherCopy;

      // If this happens, for simplicity we remove all factors that that contain a timestamp older than the smoother lag
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
      std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Latest timestamp in new factors: " << latestTimeStamp << COLOR_END
                << std::endl;
      // Filter out the factor that caused the error -------------------------
      bool filteredOutAtLeastOneKey = false;
      // Containers
      gtsam::NonlinearFactorGraph newGraphFactorsFiltered = gtsam::NonlinearFactorGraph();
      // Filtering the keyTimestampMap
      std::map<gtsam::Key, double> newGraphKeysTimeStampMapFiltered = newGraphKeysTimeStampMap;
      // For loop
      for (auto factor : newGraphFactors) {
        bool factorContainsExistentKeys = true;
        for (auto key : factor->keys()) {
          if (newGraphKeysTimeStampMap.find(key) != newGraphKeysTimeStampMap.end()) {
            if (newGraphKeysTimeStampMap.at(key) < latestTimeStamp - graphConfigPtr_->realTimeSmootherLag_) {
              std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                        << " Factor contains key older than smoother lag: " << gtsam::Symbol(key) << COLOR_END << std::endl;
              factorContainsExistentKeys = false;
              filteredOutAtLeastOneKey = true;
              // Erase from keyTimestampMap
              // newGraphKeysTimeStampMapFiltered.erase(key);
            }
          }
        }
        // Add factor if it does not contain any key older than the smoother lag
        if (factorContainsExistentKeys) {
          newGraphFactorsFiltered.add(factor);
          std::cout << YELLOW_START << "GMsf-ISAM2" << GREEN_START << " Factor added to new graph." << COLOR_END << std::endl;
        }
      }

      // Limit depth of recursion
      if (filteredOutAtLeastOneKey) {
        std::cout << YELLOW_START << "GMsf-ISAM2" << GREEN_START
                  << " Filtered out factors that are older than the smoother lag. Trying to optimize again." << COLOR_END << std::endl;
        return update(newGraphFactorsFiltered, newGraphValues, newGraphKeysTimeStampMapFiltered);
      } else {
        // Show all values and corresponding timestamps that are not in timestamp map
        for (const auto& value : newGraphValues) {
          if (newGraphKeysTimeStampMap.find(value.key) == newGraphKeysTimeStampMap.end()) {
            std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Value: " << gtsam::Symbol(value.key) << " with timestamp: "
                      << "not in timestamp map" << COLOR_END << std::endl;
          }
        }
        // Show all factors and corresponding timestamps that are not in timestamp map
        for (const auto& factor : newGraphFactors) {
          for (const auto& key : factor->keys()) {
            if (newGraphKeysTimeStampMap.find(key) == newGraphKeysTimeStampMap.end()) {
              std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Factor: " << gtsam::Symbol(key) << " with timestamp: "
                        << "not in timestamp map" << COLOR_END << std::endl;
            }
          }
        }

        std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                  << " Could not filter out any factors that are older than the smoother lag. Aborting optimization." << COLOR_END
                  << std::endl;
        return false;
      }
    } catch (const gtsam::ValuesKeyDoesNotExist& valuesKeyDoesNotExistException) {  // CASE 2 : ValuesKeyDoesNotExist exception ------------
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
                      << ". --> Removed." << COLOR_END << std::endl;
            factorContainsExistentKeys = false;
            removedAtLeastOneFactorOrKey = true;
            break;
          }
        }
        if (factorContainsExistentKeys) {
          newGraphFactorsFiltered.add(factor);
        }
      }

      // Filter out the key from the keyTimestampMap
      std::map<gtsam::Key, double> newGraphKeysTimeStampMapFiltered;
      for (auto keyTimeStamp : newGraphKeysTimeStampMap) {
        if (keyTimeStamp.first != valuesKeyNotExistent) {
          newGraphKeysTimeStampMapFiltered.insert(keyTimeStamp);
        } else {
          std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                    << " GraphKeysTimeStampMap contains non-existent key: " << gtsam::Symbol(keyTimeStamp.first) << ". --> Removed."
                    << COLOR_END << std::endl;
          removedAtLeastOneFactorOrKey = true;
        }
      }

      // Filtering the keyTimestampMap
      //      std::map<gtsam::Key, double> newGraphKeysTimeStampMapFiltered = newGraphKeysTimeStampMap;
      //      newGraphKeysTimeStampMapFiltered.erase(valuesKeyNotExistent);

      // Potentially try to optimize again
      if (removedAtLeastOneFactorOrKey) {
        std::cout << YELLOW_START << "GMsf-ISAM2" << GREEN_START
                  << " Filtered out the factor or key that caused the error (i.e. containing the key "
                  << gtsam::Symbol(valuesKeyNotExistent) << "). Trying to optimize again." << COLOR_END << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
        return update(newGraphFactorsFiltered, newGraphValues, newGraphKeysTimeStampMapFiltered);
      } else {
        std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                  << " Could not filter out any factors that caused the error (i.e. containing the key "
                  << gtsam::Symbol(valuesKeyNotExistent) << "). Aborting optimization." << COLOR_END << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
        return false;
      }
    } catch (const std::runtime_error& runtimeError) {  // CASE 3 : Runtime error ------------------------------------------------
      std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Runtime error while optimizing graph: " << runtimeError.what()
                << COLOR_END << std::endl;
      throw std::runtime_error(runtimeError.what());
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

  // Calculate State at Key
  template <class ESTIMATE_TYPE>
  ESTIMATE_TYPE calculateEstimate(const gtsam::Key& key) {
    return fixedLagSmootherPtr_->calculateEstimate<ESTIMATE_TYPE>(key);
  }
  gtsam::Pose3 calculateEstimatedPose(const gtsam::Key& key) override { return fixedLagSmootherPtr_->calculateEstimate<gtsam::Pose3>(key); }
  gtsam::Vector3 calculateEstimatedVelocity(const gtsam::Key& key) override {
    return fixedLagSmootherPtr_->calculateEstimate<gtsam::Vector3>(key);
  }
  gtsam::imuBias::ConstantBias calculateEstimatedBias(const gtsam::Key& key) override {
    return fixedLagSmootherPtr_->calculateEstimate<gtsam::imuBias::ConstantBias>(key);
  }
  gtsam::Point3 calculateEstimatedDisplacement(const gtsam::Key& key) override {
    return fixedLagSmootherPtr_->calculateEstimate<gtsam::Point3>(key);
  }

  gtsam::Vector calculateStateAtKey(const gtsam::Key& key) override { return fixedLagSmootherPtr_->calculateEstimate<gtsam::Vector>(key); }

  // Marginal Covariance
  gtsam::Matrix marginalCovariance(const gtsam::Key& key) override { return fixedLagSmootherPtr_->marginalCovariance(key); }

 private:
  // Optimizer itself
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> fixedLagSmootherPtr_;
  // Result
  gtsam::Values fixedLagSmootherOptimizedResult_;
};

}  // namespace graph_msf

#endif  // OPTIMIZER_ISAM2_FIXED_LAG_HPP
