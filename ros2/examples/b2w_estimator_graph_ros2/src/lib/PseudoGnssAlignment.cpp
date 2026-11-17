/*
Copyright 2026 by Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include "b2w_estimator_graph_ros2/PseudoGnssAlignment.h"

// std
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace b2w_se {

void PseudoGnssAlignment::add(const double t, const Eigen::Vector3d& p_M, const Eigen::Vector3d& p_W, const double sigmaXY,
                              const std::uint64_t frameGen) {
  if (!pairs_.empty() && frameGen != frameGen_) {
    pairs_.clear();  // the odometry map frame changed: the old pairs live in another frame
  }
  frameGen_ = frameGen;
  const double s = std::max(sigmaXY, params_.minSigma);
  pairs_.push_back({t, p_M, p_W, 1.0 / (s * s)});
  while (!pairs_.empty() && pairs_.front().t < t - params_.windowSec) {
    pairs_.pop_front();
  }
}

bool PseudoGnssAlignment::solve(Result& out) const {
  out = Result{};
  const int n = static_cast<int>(pairs_.size());
  out.numFixes = n;
  if (n < std::max(params_.minFixes, 3)) {
    return false;
  }
  for (int i = 1; i < n; ++i) {
    out.pathLength += (pairs_[i].p_M - pairs_[i - 1].p_M).head<2>().norm();
  }
  if (out.pathLength < params_.minPathLength) {
    return false;
  }

  const double thr = params_.inlierThreshold;
  auto residualXY = [](const Pair& p, const double yaw, const Eigen::Vector3d& t) {
    const Eigen::Vector3d q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * p.p_M + t;
    return (p.p_W - q).head<2>().norm();
  };

  // 1) 2-point RANSAC (MSAC cost), deterministic for reproducible runs. The two points must be far enough apart for a yaw.
  std::mt19937 rng(42);
  std::uniform_int_distribution<int> uni(0, n - 1);
  const double minBaseline = std::max(2.0 * thr, 0.2 * out.pathLength);
  double bestCost = std::numeric_limits<double>::infinity();
  double bestYaw = 0.0;
  Eigen::Vector3d bestT = Eigen::Vector3d::Zero();
  for (int it = 0; it < 300; ++it) {
    const Pair& pi = pairs_[uni(rng)];
    const Pair& pj = pairs_[uni(rng)];
    const Eigen::Vector2d da = (pj.p_M - pi.p_M).head<2>();
    const Eigen::Vector2d db = (pj.p_W - pi.p_W).head<2>();
    if (da.norm() < minBaseline) {
      continue;
    }
    const double yaw = std::atan2(db.y(), db.x()) - std::atan2(da.y(), da.x());
    const Eigen::Vector3d t = pi.p_W - Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * pi.p_M;
    double cost = 0.0;
    for (const Pair& p : pairs_) {
      const double r = residualXY(p, yaw, t);
      cost += std::min(r * r, thr * thr);
    }
    if (cost < bestCost) {
      bestCost = cost;
      bestYaw = yaw;
      bestT = t;
    }
  }
  if (!std::isfinite(bestCost)) {
    return false;  // no pair with a long enough baseline
  }

  // 2) Weighted least squares on the inliers (twice: the refit can change the inlier set).
  double yaw = bestYaw;
  Eigen::Vector3d t = bestT;
  std::vector<char> inlier(n, 0);
  for (int round = 0; round < 2; ++round) {
    double sw = 0.0;
    Eigen::Vector3d ma = Eigen::Vector3d::Zero(), mb = Eigen::Vector3d::Zero();
    for (int k = 0; k < n; ++k) {
      inlier[k] = residualXY(pairs_[k], yaw, t) < thr ? 1 : 0;
      if (inlier[k]) {
        sw += pairs_[k].w;
        ma += pairs_[k].w * pairs_[k].p_M;
        mb += pairs_[k].w * pairs_[k].p_W;
      }
    }
    if (sw <= 0.0) {
      return false;
    }
    ma /= sw;
    mb /= sw;
    double num = 0.0, den = 0.0;
    for (int k = 0; k < n; ++k) {
      if (!inlier[k]) continue;
      const Eigen::Vector2d a = (pairs_[k].p_M - ma).head<2>();
      const Eigen::Vector2d b = (pairs_[k].p_W - mb).head<2>();
      num += pairs_[k].w * (a.x() * b.y() - a.y() * b.x());
      den += pairs_[k].w * a.dot(b);
    }
    yaw = std::atan2(num, den);
    t = mb - Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * ma;
  }

  // 3) Quality: yaw variance = (weighted residual variance per unit weight) / (weighted spread of the path)
  double sw = 0.0, swr2 = 0.0, sr2 = 0.0, spread = 0.0;
  Eigen::Vector2d ma = Eigen::Vector2d::Zero();
  int nIn = 0;
  for (int k = 0; k < n; ++k) {
    inlier[k] = residualXY(pairs_[k], yaw, t) < thr ? 1 : 0;
    if (!inlier[k]) continue;
    ++nIn;
    sw += pairs_[k].w;
    ma += pairs_[k].w * pairs_[k].p_M.head<2>();
  }
  if (nIn < 3) {
    return false;
  }
  ma /= sw;
  for (int k = 0; k < n; ++k) {
    if (!inlier[k]) continue;
    const double r = residualXY(pairs_[k], yaw, t);
    swr2 += pairs_[k].w * r * r;
    sr2 += r * r;
    spread += pairs_[k].w * (pairs_[k].p_M.head<2>() - ma).squaredNorm();
  }
  const double varUnit = swr2 / std::max(1, 2 * nIn - 3);  // 2 residual components per fix, 3 parameters (yaw, x, y)
  out.yawStdDeg = spread > 0.0 ? std::sqrt(varUnit / spread) * 180.0 / M_PI : 180.0;
  out.rmsInlier = std::sqrt(sr2 / nIn);
  out.numInliers = nIn;
  out.T_W_M = Eigen::Isometry3d::Identity();
  out.T_W_M.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  out.T_W_M.translation() = t;

  return nIn >= params_.minFixes && 2 * nIn >= n && out.yawStdDeg <= params_.maxYawStdDeg;
}

}  // namespace b2w_se
