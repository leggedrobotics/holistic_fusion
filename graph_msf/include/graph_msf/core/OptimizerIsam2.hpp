/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef OptimizerIsam2_HPP
#define OptimizerIsam2_HPP

// GTSAM
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// Workspace
#include <graph_msf/core/Optimizer.h>

namespace graph_msf {

class OptimizerIsam2 : public Optimizer {
 public:
  explicit OptimizerIsam2(const std::shared_ptr<GraphConfig> graphConfigPtr) : Optimizer(graphConfigPtr) {
    // Standard ISAM2 Parameters
    isam2Params_.findUnusedFactorSlots = graphConfigPtr_->findUnusedFactorSlotsFlag;
    isam2Params_.enableDetailedResults = graphConfigPtr_->enableDetailedResultsFlag;
    isam2Params_.relinearizeSkip = graphConfigPtr_->relinearizeSkip;
    isam2Params_.enableRelinearization = graphConfigPtr_->enableRelinearizationFlag;
    isam2Params_.evaluateNonlinearError = graphConfigPtr_->evaluateNonlinearErrorFlag;
    isam2Params_.cacheLinearizedFactors = graphConfigPtr_->cacheLinearizedFactorsFlag;
    isam2Params_.enablePartialRelinearizationCheck = graphConfigPtr_->enablePartialRelinearizationCheckFlag;

    // Set graph re-linearization thresholds - must be lower-case letters,
    // check:gtsam::symbol_shorthand
    gtsam::FastMap<char, gtsam::Vector> relinTh;
    /// Pose
    relinTh['x'] =
        (gtsam::Vector(6) << graphConfigPtr_->rotationReLinTh, graphConfigPtr_->rotationReLinTh, graphConfigPtr_->rotationReLinTh,
         graphConfigPtr_->positionReLinTh, graphConfigPtr_->positionReLinTh, graphConfigPtr_->positionReLinTh)
            .finished();
    /// Velocity
    relinTh['v'] =
        (gtsam::Vector(3) << graphConfigPtr_->velocityReLinTh, graphConfigPtr_->velocityReLinTh, graphConfigPtr_->velocityReLinTh)
            .finished();
    /// Biases
    relinTh['b'] = (gtsam::Vector(6) << graphConfigPtr_->accBiasReLinTh, graphConfigPtr_->accBiasReLinTh, graphConfigPtr_->accBiasReLinTh,
                    graphConfigPtr_->gyroBiasReLinTh, graphConfigPtr_->gyroBiasReLinTh, graphConfigPtr_->gyroBiasReLinTh)
                       .finished();
    if (graphConfigPtr_->optimizeFixedFramePosesWrtWorld) {
      relinTh['t'] =
          (gtsam::Vector(6) << graphConfigPtr_->fixedFrameReLinTh, graphConfigPtr_->fixedFrameReLinTh, graphConfigPtr_->fixedFrameReLinTh,
           graphConfigPtr_->fixedFrameReLinTh, graphConfigPtr_->fixedFrameReLinTh, graphConfigPtr_->fixedFrameReLinTh)
              .finished();
    }
    if (graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffset) {
      relinTh['d'] = (gtsam::Vector(3) << graphConfigPtr_->displacementReLinTh, graphConfigPtr_->displacementReLinTh,
                      graphConfigPtr_->displacementReLinTh)
                         .finished();
    }
    isam2Params_.relinearizeThreshold = relinTh;

    // Factorization
    if (graphConfigPtr_->usingCholeskyFactorizationFlag) {
      isam2Params_.factorization = gtsam::ISAM2Params::CHOLESKY;  // CHOLESKY:Fast but non-stable
    } else {
      isam2Params_.factorization = gtsam::ISAM2Params::QR;  // QR:Slower but more stable im poorly
                                                            // conditioned problems
    }

    // Performance parameters
    // Set graph relinearization skip
    isam2Params_.relinearizeSkip = graphConfigPtr_->relinearizeSkip;
    // Set relinearization
    isam2Params_.enableRelinearization = graphConfigPtr_->enableRelinearizationFlag;
    // Enable Nonlinear Error
    isam2Params_.evaluateNonlinearError = graphConfigPtr_->evaluateNonlinearErrorFlag;
    // Cache linearized factors
    isam2Params_.cacheLinearizedFactors = graphConfigPtr_->cacheLinearizedFactorsFlag;
    // Enable particular relinearization check
    isam2Params_.enablePartialRelinearizationCheck = graphConfigPtr_->enablePartialRelinearizationCheckFlag;

    // Wildfire parameters
    isam2Params_.optimizationParams = gtsam::ISAM2GaussNewtonParams(graphConfigPtr_->gaussNewtonWildfireThreshold);

    // Initialize Smoother -----------------------------------------------
    fixedLagSmootherPtr_ =
        std::make_shared<gtsam::IncrementalFixedLagSmoother>(graphConfigPtr_->smootherLag,
                                                             isam2Params_);  // std::make_shared<gtsam::ISAM2>(isamParams_);
    fixedLagSmootherPtr_->params().print("GraphMSF: Factor Graph Parameters of global graph.");
  }
  ~OptimizerIsam2() = default;

  bool update() override {
    fixedLagSmootherPtr_->update();
    return true;
  }

  bool update(const gtsam::NonlinearFactorGraph& newGraphFactors, const gtsam::Values& newGraphValues,
              const std::map<gtsam::Key, double>& newGraphKeysTimeStampMap) override {
    gtsam::IncrementalFixedLagSmoother fixedLagSmootherCopy = *fixedLagSmootherPtr_;
    try {
      fixedLagSmootherCopy.update(newGraphFactors, newGraphValues, newGraphKeysTimeStampMap);
    } catch (const std::out_of_range& outOfRangeExeception) {  // Not specifically catching
      std::cerr << YELLOW_START << "GMsf-ISAM2" << RED_START
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
      std::cerr << YELLOW_START << "GMsf-ISAM2" << RED_START
                << " Values Key Does Not Exist exeception while optimizing graph: " << valuesKeyDoesNotExistException.what() << COLOR_END
                << std::endl;
      std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START
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
            std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Factor contains non-existent key: " << gtsam::Symbol(key)
                      << COLOR_END << std::endl;
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
      std::cout << YELLOW_START << "GMsf-ISAM2" << GREEN_START
                << " Filtered out the factor that caused the error. Trying to optimize again." << COLOR_END << std::endl;
      std::cout << "----------------------------------------------------------" << std::endl;
      return update(newGraphFactorsFiltered, newGraphValues, newGraphKeysTimeStampMapFiltered);
    } catch (const std::runtime_error& runtimeError) {
      std::cout << YELLOW_START << "GMsf-ISAM2" << RED_START << " Runtime error while optimizing graph: " << runtimeError.what()
                << COLOR_END << std::endl;
      throw std::runtime_error(runtimeError.what());
      return false;
    }
    *fixedLagSmootherPtr_ = fixedLagSmootherCopy;
    return true;
  }

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
  gtsam::Matrix marginalCovariance(const gtsam::Key& key) override { return fixedLagSmootherPtr_->marginalCovariance(key); }

 private:
  // Optimizer itself
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> fixedLagSmootherPtr_;
  // Parameters
  gtsam::ISAM2Params isam2Params_;
};

}  // namespace graph_msf

#endif  // OptimizerIsam2_HPP
