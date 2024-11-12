/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// GTSAM
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

// Workspace
#include "graph_msf/core/DynamicTransformDictionary.h"

#ifndef DYNAMIC_DICTIONARY_CONTAINER_H
#define DYNAMIC_DICTIONARY_CONTAINER_H

namespace graph_msf {

class DynamicDictionaryContainer {
 public:
  DynamicDictionaryContainer() = default;
  ~DynamicDictionaryContainer() = default;

  // Get the right dictionary based on template type
  template <class GTSAM_DYNAMIC_STATE_TYPE>
  DynamicTransformDictionary<GTSAM_DYNAMIC_STATE_TYPE>& get() {
    constexpr bool isPose3 = std::is_same<GTSAM_DYNAMIC_STATE_TYPE, gtsam::Pose3>::value;
    constexpr bool isPoint3 = std::is_same<GTSAM_DYNAMIC_STATE_TYPE, gtsam::Point3>::value;
    static_assert(isPose3 || isPoint3, "Type not supported.");
    // Return correct dictionary
    // Pose3
    if constexpr (isPose3) {
      return gtsamPose3ExpressionKeys_;
    }
    // Point3
    else if constexpr (isPoint3) {
      return gtsamPoint3ExpressionKeys_;
    }
  }

 private:
  DynamicTransformDictionary<gtsam::Pose3> gtsamPose3ExpressionKeys_;
  DynamicTransformDictionary<gtsam::Point3> gtsamPoint3ExpressionKeys_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_CLION_DYNAMICDICTIONARYCONTAINER_H
