#pragma once

// ROS 2
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <vector>

// Workspace
#include "graph_msf/interface/Terminal.h"
#include "graph_msf_ros2/constants.h"

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

/// Dump all declared parameters for the node
/// Dump all declared parameters for the node, including hidden ones
inline void dumpAllParams(std::shared_ptr<rclcpp::Node> node) {
  auto result = node->list_parameters({}, 10);  // depth = 10

  if (result.names.empty()) {
    RCLCPP_WARN(node->get_logger(),
                "No parameter is present in node %s",
                node->get_fully_qualified_name());
    return;
  }

  RCLCPP_INFO(node->get_logger(),
              "Listing %zu parameters for node %s (including hidden)",
              result.names.size(),
              node->get_fully_qualified_name());

  for (const auto &name : result.names) {
    rclcpp::Parameter param;
    if (node->get_parameter(name, param)) {
      RCLCPP_INFO(node->get_logger(),
                  "  %s = %s",
                  name.c_str(),
                  param.value_to_string().c_str());
    } else {
      // Try hidden parameters explicitly
      auto hidden = node->list_parameters({name}, 0);
      if (!hidden.names.empty() && node->get_parameter(name, param)) {
        RCLCPP_INFO(node->get_logger(),
                    "  %s (hidden) = %s",
                    name.c_str(),
                    param.value_to_string().c_str());
      } else {
        RCLCPP_INFO(node->get_logger(),
                    "  %s (declared, no value)",
                    name.c_str());
      }
    }
  }
}


// Implementation of Templating
template <typename T>
T tryGetParam(const rclcpp::Node* node, const std::string& key) {
  T value;

  if (node->get_parameter(key, value)) {
    printKey(key, value);
    // RCLCPP_INFO(node->get_logger(), "Successfully retrieved parameter: %s", key.c_str());
    // std::cout << YELLOW_START << "GraphMsfRos2 " << COLOR_END << key.c_str() << " set to: " << value << std::endl;
    return value;
  }

  if (node->get_parameter("/" + key, value)) {
    printKey("/" + key, value);
    // RCLCPP_INFO(node->get_logger(), "Successfully retrieved parameter with absolute key: %s", ("/" + key).c_str());
    // std::cout << YELLOW_START << "GraphMsfRos2 " << COLOR_END << key.c_str() << " set to: " << value << std::endl;
    return value;
  }

  RCLCPP_ERROR(node->get_logger(), "Parameter not found: %s", key.c_str());

  dumpAllParams(node);

  
  throw std::runtime_error("GraphMsfRos2 - " + key + " not specified.");
}

}  // namespace graph_msf
