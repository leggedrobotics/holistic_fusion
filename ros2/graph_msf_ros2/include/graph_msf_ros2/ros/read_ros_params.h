#pragma once

// ROS 2
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <vector>

// Workspace
#include "graph_msf/interface/Terminal.h"

namespace graph_msf {

template <typename T>
inline void printKey(const std::string& key, T value) {
  std::cout << YELLOW_START << "GraphMsfRos2 " << COLOR_END << key << " set to: " << value << std::endl;
}

template <>
inline void printKey(const std::string& key, std::vector<double> vector) {
  std::cout << YELLOW_START << "GraphMsfRos2 " << COLOR_END << key << " set to: ";
  for (const auto& element : vector) {
    std::cout << element << ",";
  }
  std::cout << std::endl;
}

// Implementation of Templating
template <typename T>
T tryGetParam(std::shared_ptr<rclcpp::Node> node, const std::string& key) {
  T value;
  RCLCPP_INFO(node->get_logger(), "Attempting to retrieve parameter: %s", key.c_str());

  if (node->get_parameter(key, value)) {
    printKey(key, value);
    RCLCPP_INFO(node->get_logger(), "Successfully retrieved parameter: %s", key.c_str());
    return value;
  }

  if (node->get_parameter("/" + key, value)) {
    printKey("/" + key, value);
    RCLCPP_INFO(node->get_logger(), "Successfully retrieved parameter with absolute key: %s", ("/" + key).c_str());
    return value;
  }

  RCLCPP_ERROR(node->get_logger(), "Parameter not found: %s", key.c_str());
  throw std::runtime_error("GraphMsfRos2 - " + key + " not specified.");
}

}  // namespace graph_msf
