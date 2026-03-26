/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef OPTIMIZER_ISAM2_FIXED_LAG_HPP
#define OPTIMIZER_ISAM2_FIXED_LAG_HPP

// GTSAM
#include <gtsam/linear/linearExceptions.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <set>
#include <sstream>

// Workspace
#include <graph_msf/core/optimizer/OptimizerIsam2.hpp>

namespace graph_msf {

class OptimizerIsam2FixedLag : public OptimizerIsam2 {
 public:
  explicit OptimizerIsam2FixedLag(const std::shared_ptr<GraphConfig> graphConfigPtr) : OptimizerIsam2(graphConfigPtr) {
    // Fixed Lag --> real-time: Define whether to use Cholesky factorization
    if (graphConfigPtr_->realTimeSmootherUseCholeskyFactorizationFlag_) {
      isam2Params_.factorization = gtsam::ISAM2Params::CHOLESKY;
      std::cout << "Using Cholesky factorization for real-time graph (ISAM2)." << std::endl;
    } else {
      isam2Params_.factorization = gtsam::ISAM2Params::QR;
      std::cout << "Using QR factorization for real-time graph (ISAM2)." << std::endl;
    }

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
              const std::map<gtsam::Key, double>& newGraphKeysTimeStampMap, const int depth = 0) {
    // Limit recursion depth to prevent infinite loops
    if (depth > 5) {
      std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START 
                << " Maximum recursion depth reached (" << depth << "). Aborting optimization." 
                << COLOR_END << std::endl;
      return false;
    }

    // Make copy of the fixedLagSmootherPtr_ to avoid changing the original graph
    gtsam::IncrementalFixedLagSmoother fixedLagSmootherCopy = *fixedLagSmootherPtr_;

    auto collectKeysFromFactors = [](const gtsam::NonlinearFactorGraph& factors) {
      std::set<gtsam::Key> retainedKeys;
      for (const auto& factor : factors) {
        if (!factor) {
          continue;
        }
        for (const gtsam::Key key : factor->keys()) {
          retainedKeys.insert(key);
        }
      }
      return retainedKeys;
    };

    auto filterValuesAndTimestamps = [&](const std::set<gtsam::Key>& retainedKeys, gtsam::Values& filteredValues,
                                         std::map<gtsam::Key, double>& filteredKeyTimeMap) {
      for (const auto& keyValue : newGraphValues) {
        if (retainedKeys.count(keyValue.key) > 0) {
          filteredValues.insert(keyValue.key, newGraphValues.at(keyValue.key));
        } else if (newGraphKeysTimeStampMap.find(keyValue.key) != newGraphKeysTimeStampMap.end()) {
          std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Values contains orphaned key after filtering: "
                    << gtsam::Symbol(keyValue.key) << ". -> Removed." << COLOR_END << std::endl;
        }
      }
      for (const auto& keyTimeStamp : newGraphKeysTimeStampMap) {
        if (retainedKeys.count(keyTimeStamp.first) > 0) {
          filteredKeyTimeMap.insert(keyTimeStamp);
        }
      }
    };

    auto collectKeysFromValues = [](const gtsam::Values& values) {
      std::set<gtsam::Key> keys;
      for (const auto& keyValue : values) {
        keys.insert(keyValue.key);
      }
      return keys;
    };

    auto collectKeysFromTimestampMap = [](const std::map<gtsam::Key, double>& timestampMap) {
      std::set<gtsam::Key> keys;
      for (const auto& keyTime : timestampMap) {
        keys.insert(keyTime.first);
      }
      return keys;
    };

    auto keyToString = [](const gtsam::Key key) {
      std::ostringstream oss;
      oss << gtsam::Symbol(key);
      return oss.str();
    };

    auto formatKeySet = [&](const std::set<gtsam::Key>& keys, const std::size_t maxKeys = 16) {
      std::ostringstream oss;
      oss << "[";
      std::size_t count = 0;
      for (const gtsam::Key key : keys) {
        if (count > 0) {
          oss << ", ";
        }
        if (count == maxKeys) {
          oss << "... +" << (keys.size() - maxKeys) << " more";
          break;
        }
        oss << keyToString(key);
        ++count;
      }
      oss << "]";
      return oss.str();
    };

    auto describeFactor = [&](const gtsam::NonlinearFactor::shared_ptr& factor, const std::size_t index) {
      std::ostringstream oss;
      oss << "#" << index << " type=";
      if (!factor) {
        oss << "<null>";
        return oss.str();
      }
      oss << typeid(*factor).name() << " keys=[";
      for (std::size_t keyIndex = 0; keyIndex < factor->keys().size(); ++keyIndex) {
        if (keyIndex > 0) {
          oss << ", ";
        }
        oss << keyToString(factor->keys()[keyIndex]);
      }
      oss << "]";
      return oss.str();
    };

    auto printBatchDiagnostics = [&](const std::string& reason, const gtsam::Key* focusKey = nullptr) {
      const std::set<gtsam::Key> factorKeys = collectKeysFromFactors(newGraphFactors);
      const std::set<gtsam::Key> valueKeys = collectKeysFromValues(newGraphValues);
      const std::set<gtsam::Key> timestampKeys = collectKeysFromTimestampMap(newGraphKeysTimeStampMap);
      const std::set<gtsam::Key> optimizerKeys = collectKeysFromTimestampMap(fixedLagSmootherPtr_->timestamps());

      std::set<gtsam::Key> availableKeys = optimizerKeys;
      availableKeys.insert(valueKeys.begin(), valueKeys.end());

      std::set<gtsam::Key> missingFactorKeys;
      for (const gtsam::Key factorKey : factorKeys) {
        if (availableKeys.count(factorKey) == 0) {
          missingFactorKeys.insert(factorKey);
        }
      }

      std::set<gtsam::Key> orphanValueKeys;
      for (const gtsam::Key valueKey : valueKeys) {
        if (factorKeys.count(valueKey) == 0) {
          orphanValueKeys.insert(valueKey);
        }
      }

      std::set<gtsam::Key> orphanTimestampKeys;
      for (const gtsam::Key timestampKey : timestampKeys) {
        if (factorKeys.count(timestampKey) == 0 && valueKeys.count(timestampKey) == 0) {
          orphanTimestampKeys.insert(timestampKey);
        }
      }

      std::cout << YELLOW_START << "GMsf-ISAM2" << COLOR_END << " Batch diagnostics (" << reason << ", depth=" << depth
                << "): factors=" << newGraphFactors.size() << ", values=" << newGraphValues.size()
                << ", timestamps=" << newGraphKeysTimeStampMap.size()
                << ", optimizer_keys=" << optimizerKeys.size() << std::endl;
      std::cout << YELLOW_START << "GMsf-ISAM2" << COLOR_END << " Batch factor keys: " << formatKeySet(factorKeys) << std::endl;
      if (!missingFactorKeys.empty()) {
        std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                  << " Factor keys missing from optimizer+incoming-values: " << formatKeySet(missingFactorKeys) << COLOR_END
                  << std::endl;
      }
      if (!orphanValueKeys.empty()) {
        std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Incoming values not referenced by any factor: "
                  << formatKeySet(orphanValueKeys) << COLOR_END << std::endl;
      }
      if (!orphanTimestampKeys.empty()) {
        std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Incoming timestamps not referenced by any factor/value: "
                  << formatKeySet(orphanTimestampKeys) << COLOR_END << std::endl;
      }

      if (focusKey != nullptr) {
        auto printKeyAvailability = [&](const gtsam::Key queryKey, const std::string& label) {
          std::cout << YELLOW_START << "GMsf-ISAM2" << COLOR_END << " " << label << " " << keyToString(queryKey)
                    << ": in_factors=" << (factorKeys.count(queryKey) > 0) << ", in_values=" << (valueKeys.count(queryKey) > 0)
                    << ", in_timestamps=" << (timestampKeys.count(queryKey) > 0)
                    << ", in_optimizer=" << (optimizerKeys.count(queryKey) > 0) << std::endl;
        };

        printKeyAvailability(*focusKey, "Focus key");
        const gtsam::Symbol focusSymbol(*focusKey);
        if (focusSymbol.chr() == 'x') {
          const std::uint64_t keyIndex = focusSymbol.index();
          printKeyAvailability(gtsam::symbol_shorthand::V(keyIndex), "Companion velocity");
          printKeyAvailability(gtsam::symbol_shorthand::B(keyIndex), "Companion bias");
        }

        std::size_t numPrintedFactors = 0;
        for (std::size_t factorIndex = 0; factorIndex < newGraphFactors.size(); ++factorIndex) {
          const auto& factor = newGraphFactors[factorIndex];
          if (!factor) {
            continue;
          }
          bool touchesFocusKey = false;
          for (const gtsam::Key factorKey : factor->keys()) {
            if (factorKey == *focusKey) {
              touchesFocusKey = true;
              break;
            }
          }
          if (touchesFocusKey) {
            std::cout << YELLOW_START << "GMsf-ISAM2" << COLOR_END << " Factor touching " << keyToString(*focusKey) << ": "
                      << describeFactor(factor, factorIndex) << std::endl;
            ++numPrintedFactors;
          }
        }
        if (numPrintedFactors == 0) {
          std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " No incoming factor directly touches "
                    << keyToString(*focusKey) << "." << COLOR_END << std::endl;
        }
      }
    };

    // Try to update
    try {
      fixedLagSmootherPtr_->update(newGraphFactors, newGraphValues, newGraphKeysTimeStampMap);
    }
    // Case 1: Catching the failure of marginalization (as some states which should be marginalized have not been optimized before)
    catch (const std::out_of_range& outOfRangeException) {
      std::cerr << YELLOW_START << "GMsf-ISAM2" << RED_START << " OutOfRange-Exception while optimizing graph: '"
                << outOfRangeException.what() << "'." << COLOR_END << std::endl;
      std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                << " This happens if the FixedLagSmoother tries to marginalize out states that have never been optimized before. This e.g. "
                   "happens if the graph hasn't been optimized for the duration of the smoother lag. Increase the lag or optimization "
                   "frequency in this case."
                << COLOR_END << std::endl;
      printBatchDiagnostics("OutOfRange during fixed-lag update");

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
      const std::set<gtsam::Key> retainedKeys = collectKeysFromFactors(newGraphFactorsFiltered);
      gtsam::Values newGraphValuesFiltered;
      std::map<gtsam::Key, double> newGraphKeysTimeStampMapFiltered;
      filterValuesAndTimestamps(retainedKeys, newGraphValuesFiltered, newGraphKeysTimeStampMapFiltered);

      // Limit depth of recursion
      // Try again
      if (filteredOutAtLeastOneKey) {
        std::cout << YELLOW_START << "GMsf-ISAM2" << GREEN_START
                  << " Filtered out factors that are either older than the smoother lag or coming from the future. Trying to optimize "
                     "again."
                  << COLOR_END << std::endl;
        std::cout << YELLOW_START << "GMsf-ISAM2" << COLOR_END << " Retry batch sizes after filtering: factors="
                  << newGraphFactorsFiltered.size() << ", values=" << newGraphValuesFiltered.size()
                  << ", timestamps=" << newGraphKeysTimeStampMapFiltered.size() << std::endl;
        *fixedLagSmootherPtr_ = fixedLagSmootherCopy;
        return update(newGraphFactorsFiltered, newGraphValuesFiltered, newGraphKeysTimeStampMapFiltered, depth + 1);
      }
      // Nothing changed, hence abort
      else {
        // Show all values and corresponding timestamps that are not in timestamp map
        std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                  << " Could not filter out any factors that are either older than the smoother lag or coming from the future. Aborting "
                     "optimization."
                  << COLOR_END << std::endl;
        *fixedLagSmootherPtr_ = fixedLagSmootherCopy;
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
      const gtsam::Key valuesKeyNotExistent = valuesKeyDoesNotExistException.key();
      printBatchDiagnostics("ValuesKeyDoesNotExist", &valuesKeyNotExistent);

      // Filter out the factor that caused the error -------------------------
      gtsam::NonlinearFactorGraph newGraphFactorsFiltered;
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
      if (newGraphKeysTimeStampMap.find(valuesKeyNotExistent) != newGraphKeysTimeStampMap.end()) {
        std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                  << " GraphKeysTimeStampMap contains non-existent key: " << gtsam::Symbol(valuesKeyNotExistent) << ". -> Removed."
                  << COLOR_END << std::endl;
        removedAtLeastOneFactorOrKey = true;
      }
      if (newGraphValues.exists(valuesKeyNotExistent)) {
        std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Values contains non-existent key: "
                  << gtsam::Symbol(valuesKeyNotExistent) << ". -> Removed." << COLOR_END << std::endl;
        removedAtLeastOneFactorOrKey = true;
      }
      const std::set<gtsam::Key> retainedKeys = collectKeysFromFactors(newGraphFactorsFiltered);
      gtsam::Values newGraphValuesFiltered;
      std::map<gtsam::Key, double> newGraphKeysTimeStampMapFiltered;
      filterValuesAndTimestamps(retainedKeys, newGraphValuesFiltered, newGraphKeysTimeStampMapFiltered);

      // Potentially try to optimize again
      if (removedAtLeastOneFactorOrKey) {
        std::cout << YELLOW_START << "GMsf-ISAM2" << GREEN_START
                  << " Filtered out the factor or key that caused the error (i.e. containing the key "
                  << gtsam::Symbol(valuesKeyNotExistent) << "). Trying to optimize again." << COLOR_END << std::endl;
        std::cout << YELLOW_START << "GMsf-ISAM2" << COLOR_END << " Retry batch sizes after filtering: factors="
                  << newGraphFactorsFiltered.size() << ", values=" << newGraphValuesFiltered.size()
                  << ", timestamps=" << newGraphKeysTimeStampMapFiltered.size() << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
        // Copy back
        *fixedLagSmootherPtr_ = fixedLagSmootherCopy;
        // Try again
        return update(newGraphFactorsFiltered, newGraphValuesFiltered, newGraphKeysTimeStampMapFiltered, depth + 1);
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
      *fixedLagSmootherPtr_ = fixedLagSmootherCopy;
      throw std::runtime_error(valuesKeyAlreadyExistsException.what());
    }
    // Case 4: Runtime error --> can't handle explicitly
    catch (const std::runtime_error& runtimeError) {
      std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Runtime error while optimizing graph: " << runtimeError.what()
                << COLOR_END << std::endl;
      *fixedLagSmootherPtr_ = fixedLagSmootherCopy;
      throw std::runtime_error(runtimeError.what());
    }
    // Case 5: Typical indeterminant linear system
    catch (const gtsam::IndeterminantLinearSystemException& indeterminantLinearSystemException) {
      std::cerr << YELLOW_START << "GMsf-ISAM2" << RED_START << " IndeterminantLinearSystem exception while optimizing graph: '"
                << indeterminantLinearSystemException.what() << "'" << COLOR_END << std::endl;
      std::cout << YELLOW_START << "GMsf-ISAM2" << COLOR_END
                << " This happens if the linear system is indeterminant, which usually indicates that there are not enough constraints to "
                   "solve for all variables."
                << std::endl;

      // Get key
      gtsam::Key key = indeterminantLinearSystemException.nearbyVariable();
      std::cout << YELLOW_START << "GMsf-ISAM2" << COLOR_END << " The key causing the issue is: " << gtsam::Symbol(key) << std::endl;
      printBatchDiagnostics("IndeterminantLinearSystemException", &key);

      // If bias --> fix graph
      const gtsam::Symbol symbol(key);
      const char stateCategory = symbol.chr();
      if (stateCategory == 'b') {
        const gtsam::Key& biasKey = key;  // Alias
        std::cout << YELLOW_START << "GMsf-ISAM2" << COLOR_END << " Latest estimated bias for key " << symbol << ": "
                  << this->latestImuBias_ << std::endl;
        // Get number component of symbol
        const int symbolNumber = symbol.index();
        gtsam::Key poseKey = gtsam::symbol_shorthand::X(symbolNumber);
        std::cout << YELLOW_START << "GMsf-ISAM2" << COLOR_END << " Latest estimated pose for key " << gtsam::Symbol(poseKey) << ": "
                  << this->latestPose_ << std::endl;
        // Uncertainty
        const auto priorBiasNoise =
            gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << graphConfigPtr_->initialAccBiasNoiseDensity_,  // m/s^2
                                                 graphConfigPtr_->initialAccBiasNoiseDensity_,                      // m/s^2
                                                 graphConfigPtr_->initialAccBiasNoiseDensity_,                      // m/s^2
                                                 graphConfigPtr_->initialGyroBiasNoiseDensity_,                     // rad/s
                                                 graphConfigPtr_->initialGyroBiasNoiseDensity_,                     // rad/s
                                                 graphConfigPtr_->initialGyroBiasNoiseDensity_)                     // rad/s
                                                    .finished());  // acc, acc, acc, gyro, gyro, gyro

        const auto priorPoseNoise =
            gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << graphConfigPtr_->initialOrientationNoiseDensity_,  // rad
                                                 graphConfigPtr_->initialOrientationNoiseDensity_,                      // rad
                                                 graphConfigPtr_->initialOrientationNoiseDensity_,                      // rad
                                                 graphConfigPtr_->initialPositionNoiseDensity_,                         // m
                                                 graphConfigPtr_->initialPositionNoiseDensity_,                         // m
                                                 graphConfigPtr_->initialPositionNoiseDensity_)                         // m
                                                    .finished());
        // Add additional factor for bias
        gtsam::NonlinearFactorGraph newGraphFactorsExtended = newGraphFactors;
        newGraphFactorsExtended.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(biasKey, this->latestImuBias_, priorBiasNoise));
        newGraphFactorsExtended.add(gtsam::PriorFactor<gtsam::Pose3>(poseKey, this->latestPose_, priorPoseNoise));
        // Copy back
        *fixedLagSmootherPtr_ = fixedLagSmootherCopy;
        // Recursion
        return update(newGraphFactorsExtended, newGraphValues, newGraphKeysTimeStampMap, depth + 1);
      } else {
        std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
                  << " The key causing the issue is not a bias key. Currently, only bias keys can be fixed automatically. Aborting "
                     "optimization."
                  << COLOR_END << std::endl;
        *fixedLagSmootherPtr_ = fixedLagSmootherCopy;
        return false;
      }
      throw std::runtime_error(indeterminantLinearSystemException.what());
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
