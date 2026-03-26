/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef OPTIMIZER_LM_HPP
#define OPTIMIZER_LM_HPP

// GTSAM
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Workspace
#include <graph_msf/core/optimizer/OptimizerBase.h>

namespace graph_msf {

class OptimizerLM : public OptimizerBase {
 public:
  explicit OptimizerLM(const std::shared_ptr<GraphConfig> graphConfigPtr) : OptimizerBase(graphConfigPtr) {
    // Standard LM Parameters
    lmParams_.setVerbosity("ERROR");
    gtsam::LevenbergMarquardtParams::SetCeresDefaults(&lmParams_);

    // Set Custom LM Parameters
    //    lmParams_.setLinearSolverType(
    //        "CHOLMOD");  // "MULTIFRONTAL_CHOLESKY", "MULTIFRONTAL_QR", "SEQUENTIAL_CHOLESKY", "SEQUENTIAL_QR", "ITERATIVE", "CHOLMOD"
    lmParams_.setMaxIterations(50);       // Ceres-like default: 50
    lmParams_.setRelativeErrorTol(1e-6);  // Ceres-like default: 1e-6
    lmParams_.setAbsoluteErrorTol(0.0);   // Ceres-like default: 0.0

    lmParams_.setLinearSolverType("MULTIFRONTAL_QR");  // Ceres-like leaves inherited default unchanged: "MULTIFRONTAL_CHOLESKY"
    // switch to "MULTIFRONTAL_QR" if conditioning is problematic

    lmParams_.setOrderingType("COLAMD");  // METIS //NATURAL   // Ceres-like leaves inherited default unchanged: "COLAMD"
    // for landmark-heavy problems, prefer a custom constrained ordering:
    // keep states last, eliminate landmarks first

    lmParams_.setDiagonalDamping(true);        // Ceres-like default: true
    lmParams_.setUseFixedLambdaFactor(false);  // Ceres-like default: false
    lmParams_.setlambdaInitial(1e-4);          // Ceres-like default: 1e-4
    lmParams_.setlambdaFactor(2.0);            // Ceres-like default: 2.0
    lmParams_.setlambdaLowerBound(1e-16);      // Ceres-like default: 1e-16
    lmParams_.setlambdaUpperBound(1e32);       // Ceres-like default: 1e32

    lmParams_.minModelFidelity = 1e-3;  // Accept LM step only if modelFidelity >= this threshold. Default in Ceres-like GTSAM LM: 1e-3.
    // Raise to 1e-2..1e-1 for more conservative, more robust step acceptance; lower to 1e-4 if LM is overly cautious and converges too
    // slowly       // Ceres-like default: 1e-3
    //    lmParams_.minModelFidelity = 1e-1;
  }
  ~OptimizerLM() = default;

  // Implementation
  bool updateExistingValues(const gtsam::Values& newGraphValues) override {
    // Update Values
    throw std::logic_error("GraphMSF: OptimizerLM: updateExistingValues: Not implemented.");
  }

 protected:
  // Parameters
  gtsam::LevenbergMarquardtParams lmParams_;
};

}  // namespace graph_msf

#endif  // OPTIMIZER_LM_HPP
