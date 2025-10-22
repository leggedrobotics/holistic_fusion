#pragma once

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>
#include <iostream>

#include "graph_msf/interface/Terminal.h"
#include "graph_msf_ros2/constants.h"

namespace graph_msf {

// ---------- printing ----------
template <typename T>
inline void printKey(const std::string& key, const T& value) {
  std::cout << YELLOW_START << "GraphMsfRos2 " << COLOR_END << key << " set to: " << value << std::endl;
}

template <>
inline void printKey<std::vector<double>>(const std::string& key, const std::vector<double>& vec) {
  std::cout << YELLOW_START << "GraphMsfRos2 " << COLOR_END << key << " set to: ";
  for (size_t i = 0; i < vec.size(); ++i) {
    std::cout << vec[i] << (i + 1 < vec.size() ? "," : "");
  }
  std::cout << std::endl;
}

// ---------- dumpAllParams ----------
inline void dumpAllParams(const rclcpp::Node& node) {
  auto result = node.list_parameters({}, 10);  // depth = 10
  if (result.names.empty()) {
    RCLCPP_WARN(node.get_logger(), "No parameter is present in node %s", node.get_fully_qualified_name());
    return;
  }

  RCLCPP_INFO(node.get_logger(),
              "Listing %zu parameters for node %s (including hidden)",
              result.names.size(),
              node.get_fully_qualified_name());

  for (const auto& name : result.names) {
    rclcpp::Parameter param;
    if (node.get_parameter(name, param)) {
      RCLCPP_INFO(node.get_logger(), "  %s = %s", name.c_str(), param.value_to_string().c_str());
    } else {
      // Try hidden parameters explicitly
      auto hidden = node.list_parameters({name}, 0);
      if (!hidden.names.empty() && node.get_parameter(name, param)) {
        RCLCPP_INFO(node.get_logger(), "  %s (hidden) = %s", name.c_str(), param.value_to_string().c_str());
      } else {
        RCLCPP_INFO(node.get_logger(), "  %s (declared, no value)", name.c_str());
      }
    }
  }
}

// convenience overloads
inline void dumpAllParams(const rclcpp::Node* node) { dumpAllParams(*node); }
inline void dumpAllParams(const std::shared_ptr<rclcpp::Node>& node) { dumpAllParams(*node); }

// ---------- tryGetParam (primary on Node&) ----------
template <typename T>
T tryGetParam(const rclcpp::Node& node, const std::string& key) {
  T value;
  if (node.get_parameter(key, value)) {
    printKey(key, value);
    return value;
  }
  if (node.get_parameter("/" + key, value)) {
    printKey("/" + key, value);
    return value;
  }

  RCLCPP_ERROR(node.get_logger(), "Parameter not found: %s", key.c_str());
  dumpAllParams(node);
  throw std::runtime_error("GraphMsfRos2 - " + key + " not specified.");
}

// convenience overloads
template <typename T>
T tryGetParam(const rclcpp::Node* node, const std::string& key) { return tryGetParam<T>(*node, key); }

template <typename T>
T tryGetParam(const std::shared_ptr<rclcpp::Node>& node, const std::string& key) { return tryGetParam<T>(*node, key); }

}  // namespace graph_msf
