/*
Copyright 2023 by Julian Nubert & Simon Kerscher, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef HOLISTIC_FUSION_ROS_READ_ROS_PARAMS_H
#define HOLISTIC_FUSION_ROS_READ_ROS_PARAMS_H

// Workspace
#include "holistic_fusion/interface/Terminal.h"

namespace holistic_fusion {

template <typename T>
inline void printKey(const std::string& key, T value) {
  std::cout << YELLOW_START << "HolisticFusionRos " << COLOR_END << key << "  set to: " << value << std::endl;
}

template <>
inline void printKey(const std::string& key, std::vector<double> vector) {
  std::cout << YELLOW_START << "HolisticFusionRos " << COLOR_END << key << " set to: ";
  for (const auto& element : vector) {
    std::cout << element << ",";
  }
  std::cout << std::endl;
}

// Implementation of Templating
template <typename T>
T tryGetParam(const std::string& key, const ros::NodeHandle& privateNode) {
  T value;
  if (privateNode.getParam(key, value)) {
    printKey(key, value);
    return value;
  } else if (privateNode.getParam("/" + key, value)) {
    printKey("/" + key, value);
    return value;
  } else {
    throw std::runtime_error("HolisticFusionRos - " + key + " not specified.");
  }
}

}  // namespace holistic_fusion

#endif  // HOLISTIC_FUSION_ROS_READ_ROS_PARAMS_H
