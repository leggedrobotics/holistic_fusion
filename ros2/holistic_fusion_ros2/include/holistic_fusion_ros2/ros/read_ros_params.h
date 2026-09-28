#pragma once

// ROS 2
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <vector>

// Workspace
#include "holistic_fusion/interface/Terminal.h"
#include "holistic_fusion_ros2/constants.h"

namespace holistic_fusion {

template <typename T>
inline void printKey(const std::string& key, T value) {
  std::cout << YELLOW_START << "HolisticFusionRos2 " << COLOR_END << key << " set to: " << value << std::endl;
}

template <>
inline void printKey(const std::string& key, std::vector<double> vector) {
  std::cout << YELLOW_START << "HolisticFusionRos2 " << COLOR_END << key << " set to: ";
  for (const auto& element : vector) {
    std::cout << element << ",";
  }
  std::cout << std::endl;
}

// Implementation of Templating
template <typename T>
T tryGetParam(const rclcpp::Node* node, const std::string& key) {
  T value;

  if (node->get_parameter(key, value)) {
    printKey(key, value);
   return value;
  }

  if (node->get_parameter("/" + key, value)) {
    printKey("/" + key, value);
    return value;
  }

  RCLCPP_ERROR(node->get_logger(), "Parameter not found: %s", key.c_str());
  throw std::runtime_error("HolisticFusionRos2 - " + key + " not specified.");
}

}  // namespace holistic_fusion
