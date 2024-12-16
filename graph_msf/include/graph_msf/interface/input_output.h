/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_INPUT_OUTPUT_H
#define GMSF_INPUT_OUTPUT_H

#include <boost/filesystem.hpp>

namespace graph_msf {

// Utility
std::string getLatestSubdirectory(const std::string& directoryPath) {
  boost::filesystem::path latestDir;
  std::time_t latestTime = 0;

  REGULAR_COUT << " Looking for latest directory in " << directoryPath << std::endl;

  for (const auto& entry : boost::filesystem::directory_iterator(directoryPath)) {
    if (is_directory(entry)) {
      std::time_t currentLastWriteTime = boost::filesystem::last_write_time(entry);
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
