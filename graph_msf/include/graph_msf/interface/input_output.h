/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_INPUT_OUTPUT_H
#define GMSF_INPUT_OUTPUT_H

// C++
#include <filesystem>
#include <iostream>
#include <string>

// Workspace
#include "graph_msf/interface/constants.h"

namespace graph_msf {

// Utility
inline std::string getLatestSubdirectory(const std::string& directoryPath) {
  std::filesystem::path latestDir;
  std::filesystem::file_time_type latestTime = std::filesystem::file_time_type::min();

  REGULAR_COUT << " Looking for latest directory in " << directoryPath << std::endl;

  for (const auto& entry : std::filesystem::directory_iterator(directoryPath)) {
    if (entry.is_directory()) {
      const std::filesystem::file_time_type currentLastWriteTime = entry.last_write_time();
      if (currentLastWriteTime > latestTime) {
        latestTime = currentLastWriteTime;
        latestDir = entry.path();
      }
    }
  }

  return latestDir.empty() ? "" : latestDir.filename().string();
}

}  // namespace graph_msf

#endif  // GMSF_INPUT_OUTPUT_H
