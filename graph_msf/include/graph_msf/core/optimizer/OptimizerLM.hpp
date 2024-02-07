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
  }
  ~OptimizerLM() = default;

 protected:
  // Parameters
  gtsam::LevenbergMarquardtParams lmParams_;
};

}  // namespace graph_msf

#endif  // OPTIMIZER_LM_HPP
