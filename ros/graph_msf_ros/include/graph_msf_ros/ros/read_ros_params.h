#ifndef GRAPH_MSF_ROS_READ_ROS_PARAMS_H
#define GRAPH_MSF_ROS_READ_ROS_PARAMS_H

// Workspace
#include "graph_msf/interface/Terminal.h"

namespace graph_msf {

template <typename T>
inline void printKey(const std::string& key, T value) {
  std::cout << YELLOW_START << "GraphMsfRos " << COLOR_END << key << "  set to: " << value << std::endl;
}

template <>
inline void printKey(const std::string& key, std::vector<double> vector) {
  std::cout << YELLOW_START << "GraphMsfRos " << COLOR_END << key << " set to: ";
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
    throw std::runtime_error("GraphMsfRos - " + key + " not specified.");
  }
}

}  // namespace graph_msf

#endif  // GRAPH_MSF_ROS_READ_ROS_PARAMS_H
