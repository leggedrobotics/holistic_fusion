/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
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
    isam2Params_.findUnusedFactorSlots = graphConfigPtr_->findUnusedFactorSlotsFlag_;
    isam2Params_.enableDetailedResults = graphConfigPtr_->enableDetailedResultsFlag_;
    isam2Params_.relinearizeSkip = graphConfigPtr_->relinearizeSkip_;
    isam2Params_.enableRelinearization = graphConfigPtr_->enableRelinearizationFlag_;
    isam2Params_.evaluateNonlinearError = graphConfigPtr_->evaluateNonlinearErrorFlag_;
    isam2Params_.cacheLinearizedFactors = graphConfigPtr_->cacheLinearizedFactorsFlag_;
    isam2Params_.enablePartialRelinearizationCheck = graphConfigPtr_->enablePartialRelinearizationCheckFlag_;

    // Set graph re-linearization thresholds - must be lower-case letters,
    // check:gtsam::symbol_shorthand
    gtsam::FastMap<char, gtsam::Vector> relinTh;
    /// Robot Pose
    relinTh['x'] =
        (gtsam::Vector(6) << graphConfigPtr_->rotationReLinTh_, graphConfigPtr_->rotationReLinTh_, graphConfigPtr_->rotationReLinTh_,
         graphConfigPtr_->positionReLinTh_, graphConfigPtr_->positionReLinTh_, graphConfigPtr_->positionReLinTh_)
            .finished();
    /// Robot Velocity
    relinTh['v'] =
        (gtsam::Vector(3) << graphConfigPtr_->velocityReLinTh_, graphConfigPtr_->velocityReLinTh_, graphConfigPtr_->velocityReLinTh_)
            .finished();
    /// Biases
    relinTh['b'] =
        (gtsam::Vector(6) << graphConfigPtr_->accBiasReLinTh_, graphConfigPtr_->accBiasReLinTh_, graphConfigPtr_->accBiasReLinTh_,
         graphConfigPtr_->gyroBiasReLinTh_, graphConfigPtr_->gyroBiasReLinTh_, graphConfigPtr_->gyroBiasReLinTh_)
            .finished();
    /// Reference Frames
    if (graphConfigPtr_->optimizeReferenceFramePosesWrtWorldFlag_) {
      relinTh['r'] = (gtsam::Vector(6) << graphConfigPtr_->referenceFrameReLinTh_, graphConfigPtr_->referenceFrameReLinTh_,
                      graphConfigPtr_->referenceFrameReLinTh_, graphConfigPtr_->referenceFrameReLinTh_,
                      graphConfigPtr_->referenceFrameReLinTh_, graphConfigPtr_->referenceFrameReLinTh_)
                         .finished();
    }
    /// Calibration
    if (graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffsetFlag_) {
      /// 6 DoF (Position and Rotation)
      relinTh['c'] = (gtsam::Vector(6) << graphConfigPtr_->calibrationReLinTh_, graphConfigPtr_->calibrationReLinTh_,
                      graphConfigPtr_->calibrationReLinTh_, graphConfigPtr_->calibrationReLinTh_, graphConfigPtr_->calibrationReLinTh_,
                      graphConfigPtr_->calibrationReLinTh_)
                         .finished();
      /// 3 DoF (Position Displacement)
      relinTh['d'] = (gtsam::Vector(3) << graphConfigPtr_->displacementReLinTh_, graphConfigPtr_->displacementReLinTh_,
                      graphConfigPtr_->displacementReLinTh_)
                         .finished();
    }
    /// Landmarks
    relinTh['l'] =
        (gtsam::Vector(3) << graphConfigPtr_->landmarkReLinTh_, graphConfigPtr_->landmarkReLinTh_, graphConfigPtr_->landmarkReLinTh_)
            .finished();
    /// Set relinearization thresholds
    isam2Params_.relinearizeThreshold = relinTh;

    // Performance parameters
    // Set graph relinearization skip
    isam2Params_.relinearizeSkip = graphConfigPtr_->relinearizeSkip_;
    // Set relinearization
    isam2Params_.enableRelinearization = graphConfigPtr_->enableRelinearizationFlag_;
    // Enable Nonlinear Error
    isam2Params_.evaluateNonlinearError = graphConfigPtr_->evaluateNonlinearErrorFlag_;
    // Cache linearized factors
    isam2Params_.cacheLinearizedFactors = graphConfigPtr_->cacheLinearizedFactorsFlag_;
    // Enable particular relinearization check
    isam2Params_.enablePartialRelinearizationCheck = graphConfigPtr_->enablePartialRelinearizationCheckFlag_;

    // Wildfire parameters
    isam2Params_.optimizationParams = gtsam::ISAM2GaussNewtonParams(graphConfigPtr_->gaussNewtonWildfireThreshold_);
  }
  ~OptimizerIsam2() = default;

  // Implementation
  bool updateExistingValues(const gtsam::Values& newGraphValues) override {
    // Update Values
    throw std::logic_error("GraphMSF: OptimizerIsam2: updateExistingValues: Not implemented.");
  }

 protected:
  // Parameters
  gtsam::ISAM2Params isam2Params_;
  // Flag
  bool optimizedAtLeastOnceFlag_ = false;
};

}  // namespace graph_msf

#endif  // OPTIMIZER_ISAM2_HPP
