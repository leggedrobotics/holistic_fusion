/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef OPTIMIZER_ISAM2_HPP
#define OPTIMIZER_ISAM2_HPP

// GTSAM
#include <gtsam/nonlinear/ISAM2.h>

// Workspace
#include <graph_msf/core/optimizer/OptimizerBase.h>

namespace graph_msf {

class OptimizerIsam2 : public OptimizerBase {
 public:
  explicit OptimizerIsam2(const std::shared_ptr<GraphConfig> graphConfigPtr) : OptimizerBase(graphConfigPtr) {
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
  }
  ~OptimizerIsam2() = default;

 protected:
  // Parameters
  gtsam::ISAM2Params isam2Params_;
};

}  // namespace graph_msf

#endif  // OPTIMIZER_ISAM2_HPP
