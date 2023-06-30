/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GTSAMEXPRESSIONTRANSFORMS_H
#define GTSAMEXPRESSIONTRANSFORMS_H

// GTSAM
#include <gtsam/slam/expressions.h>

// Workspace
#include "graph_msf/core/TransformsDictionary.h"

namespace graph_msf {

class GtsamExpressionTransforms : public TransformsDictionary<gtsam::Key> {
 public:
  // Constructor
  GtsamExpressionTransforms() : TransformsDictionary<gtsam::Key>() {
    std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " StaticTransforms instance created." << std::endl;
  }

  // Functionality ------------------------------------------------------------
  void addNewExpressionTransformation(const std::string& frame1, const std::string& frame2) {
    set_T_frame1_frame2(frame1, frame2, gtsam::symbol_shorthand::T(getNumberStoredTransformationPairs()));
  }

};

}  // namespace graph_msf

#endif  // STATIC_TRANSFORMS_H
