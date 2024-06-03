/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef OPTIMIZER_LM_FIXED_LAG_HPP
#define OPTIMIZER_LM_FIXED_LAG_HPP

// GTSAM
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>

// Workspace
#include <graph_msf/core/optimizer/OptimizerLM.hpp>

namespace graph_msf {

class OptimizerLMFixedLag : public OptimizerLM {
 public:
  explicit OptimizerLMFixedLag(const std::shared_ptr<GraphConfig> graphConfigPtr) : OptimizerLM(graphConfigPtr) {
    // Initialize Real-time Smoother -----------------------------------------------
    fixedLagSmootherPtr_ = std::make_shared<gtsam::BatchFixedLagSmoother>(graphConfigPtr_->realTimeSmootherLag_, lmParams_);
    // Print
    fixedLagSmootherPtr_->print("LM FixedLagSmoother initialized with lag: ");
  }
  ~OptimizerLMFixedLag() = default;

  bool update() override {
    fixedLagSmootherPtr_->update();
    return true;
  }

  bool update(const gtsam::NonlinearFactorGraph& newGraphFactors, const gtsam::Values& newGraphValues,
              const std::map<gtsam::Key, double>& newGraphKeysTimeStampMap) override {
    // Try to update
    try {
      fixedLagSmootherPtr_->update(newGraphFactors, newGraphValues, newGraphKeysTimeStampMap);
    } catch (const std::out_of_range& outOfRangeExeception) {  // Not specifically catching
      std::cerr << YELLOW_START << "GMsf-LM-FixedLagSmoother" << RED_START
                << " Out of Range exception while optimizing graph: " << outOfRangeExeception.what() << COLOR_END << std::endl;
      std::cout
          << YELLOW_START << "GMsf-ISAM2" << RED_START
          << " This usually happens if the measurement delay is larger than the graph-smootherLag, i.e. the optimized graph instances are "
             "not connected. Increase the lag in this case."
          << COLOR_END << std::endl;
      // throw std::out_of_range(outOfRangeExeception.what());
      return false;
    } catch (const gtsam::ValuesKeyDoesNotExist& valuesKeyDoesNotExistException) {  // Catching, as this can happen in practice (factors
                                                                                    // added that are outside the smoother window)
      std::cout << "----------------------------------------------------------" << std::endl;
      std::cerr << YELLOW_START << "GMsf-LM-FixedLagSmoother" << RED_START
                << " Values Key Does Not Exist exeception while optimizing graph: " << valuesKeyDoesNotExistException.what() << COLOR_END
                << std::endl;
      std::cout << YELLOW_START << "GMsf-LM-FixedLagSmoother" << RED_START
                << " This happens if a factor is added to the graph that contains a non-existent state. The most common case is a factor "
                   "containing a graph state that was marginalized out before. To avoid this, increase the factor graph lag or reduce "
                   "the delay of your measurements."
                << COLOR_END << std::endl;
      // Filter out the factor that caused the error -------------------------
      gtsam::NonlinearFactorGraph newGraphFactorsFiltered;
      gtsam::Key valuesKeyNotExistent = valuesKeyDoesNotExistException.key();
      for (auto factor : newGraphFactors) {
        bool factorContainsExistentKeys = true;
        for (auto key : factor->keys()) {
          if (key == valuesKeyNotExistent) {
            std::cout << YELLOW_START << "GMsf-LM-FixedLagSmoother" << RED_START
                      << " Factor contains non-existent key: " << gtsam::Symbol(key) << COLOR_END << std::endl;
            factorContainsExistentKeys = false;
            break;
          }
        }
        if (factorContainsExistentKeys) {
          newGraphFactorsFiltered.add(factor);
        }
      }
      // Filtering the keyTimestampMap
      std::map<gtsam::Key, double> newGraphKeysTimeStampMapFiltered = newGraphKeysTimeStampMap;
      newGraphKeysTimeStampMapFiltered.erase(valuesKeyNotExistent);
      // Try again
      std::cout << YELLOW_START << "GMsf-LM-FixedLagSmoother" << GREEN_START
                << " Filtered out the factor that caused the error. Trying to optimize again." << COLOR_END << std::endl;
      std::cout << "----------------------------------------------------------" << std::endl;
      return update(newGraphFactorsFiltered, newGraphValues, newGraphKeysTimeStampMapFiltered);
    } catch (const std::runtime_error& runtimeError) {
      std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Runtime error while optimizing graph: " << runtimeError.what()
                << COLOR_END << std::endl;
      throw std::runtime_error(runtimeError.what());
    }
    return true;
  }

  // Optimize
  void optimize(int maxIterations) override {
    // Print
    std::cout << YELLOW_START << "GraphMSF: OptimizerLMFixedLag" << GREEN_START << " Optimizing fixed-lag smoother." << COLOR_END
              << std::endl;

    // Optimize
    fixedLagSmootherOptimizedResult_ = fixedLagSmootherPtr_->calculateEstimate();
  }

  // Get Result
  const gtsam::Values& getAllOptimizedStates() override { return fixedLagSmootherOptimizedResult_; }

  // Get all keys of optimized states
  gtsam::KeyVector getAllOptimizedKeys() override { return fixedLagSmootherOptimizedResult_.keys(); }

  // Get nonlinear factor graph
  const gtsam::NonlinearFactorGraph& getNonlinearFactorGraph() const override { return fixedLagSmootherPtr_->getFactors(); }

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

  gtsam::Vector calculateStateAtKey(const gtsam::Key& key) { return fixedLagSmootherPtr_->calculateEstimate<gtsam::Vector>(key); }

  // Marginal Covariance
  gtsam::Matrix marginalCovariance(const gtsam::Key& key) override {
    // Not implemented
    std::cout << "FixedLagSmoother: Marginal Covariance not implemented yet. Returning Identity of correct size for key type: "
              << gtsam::symbolChr(key) << std::endl;
    const unsigned char measurementType = gtsam::symbolChr(key);
    // Switch case
    switch (measurementType) {
      case 'x':
        return gtsam::Matrix::Identity(6, 6);
      case 'v':
        return gtsam::Matrix::Identity(3, 3);
      case 'b':
        return gtsam::Matrix::Identity(6, 6);
      case 't':
        return gtsam::Matrix::Identity(6, 6);
      case 'd':
        return gtsam::Matrix::Identity(3, 3);
      default:
        throw std::runtime_error("FixedLagSmoother: Marginal Covariance not implemented for this key type: " + gtsam::symbolChr(key));
    }
  }

 private:
  // Optimizer itself
  std::shared_ptr<gtsam::BatchFixedLagSmoother> fixedLagSmootherPtr_;
  // Result
  gtsam::Values fixedLagSmootherOptimizedResult_;
};

}  // namespace graph_msf

#endif  // OPTIMIZER_LM_FIXED_LAG_HPP
