/*
Copyright 2026 by Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#pragma once

// std
#include <cstdint>
#include <deque>

// Eigen
#include <Eigen/Geometry>

namespace b2w_se {

// World alignment from ML pseudo-GNSS fixes (e.g. cross-view localization): sparse (~1 Hz) and metre-level noisy, unlike RTK.
// Each fix is paired with the odometry position (LIO/VIO map frame M) at its stamp. The fit is yaw + translation only (roll and
// pitch come from gravity): 2-point RANSAC, then a weighted least-squares refit on the inliers. A fit is accepted only once the
// window holds enough motion for the noise of the fixes: its yaw standard deviation is computed from the actual inlier
// residuals (not the reported sigmas) and the spread of the path, so noisier fixes automatically need a longer path.
struct PseudoGnssAlignmentParams {
  double windowSec = 180.0;       // [s] pairs older than this (w.r.t. the newest) are dropped
  double minPathLength = 10.0;    // [m] odometry path length within the window
  int minFixes = 15;              // inlier fixes
  double inlierThreshold = 6.0;   // [m] horizontal distance of a fix from the fitted path
  double maxYawStdDeg = 3.0;      // [deg] accept once the yaw standard deviation of the fit is below this
  double minSigma = 0.5;          // [m] floor of the reported sigma used for the weights
};

class PseudoGnssAlignment {
 public:
  struct Result {
    Eigen::Isometry3d T_W_M = Eigen::Isometry3d::Identity();  // yaw-only rotation
    int numFixes = 0;
    int numInliers = 0;
    double pathLength = 0.0;  // [m]
    double yawStdDeg = 0.0;   // [deg]
    double rmsInlier = 0.0;   // [m] horizontal
  };

  explicit PseudoGnssAlignment(const PseudoGnssAlignmentParams& params) : params_(params) {}

  void reset() { pairs_.clear(); }

  // p_M: odometry position of the measured point at time t; p_W: the fix in the world frame; sigmaXY: reported horizontal
  // standard deviation. frameGen identifies the odometry map frame: a new value (source switch, re-origin) drops the window.
  void add(double t, const Eigen::Vector3d& p_M, const Eigen::Vector3d& p_W, double sigmaXY, std::uint64_t frameGen);

  // Returns true if the current window gives an acceptable fit. 'out' is filled whenever a fit could be computed.
  bool solve(Result& out) const;

  [[nodiscard]] const PseudoGnssAlignmentParams& params() const { return params_; }

 private:
  struct Pair {
    double t;
    Eigen::Vector3d p_M;
    Eigen::Vector3d p_W;
    double w;  // 1 / sigma^2
  };
  PseudoGnssAlignmentParams params_;
  std::deque<Pair> pairs_;
  std::uint64_t frameGen_ = 0;
};

}  // namespace b2w_se
