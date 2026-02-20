// TrajectoryAlignment.h
/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_TrajectoryAlignment_H
#define GRAPH_MSF_TrajectoryAlignment_H

// C++
#include <Eigen/Eigen>
#include <memory>
#include <mutex>
// Package
#include "graph_msf/geometry/Trajectory.h"

// Defined macros
#define GREEN_START "\033[92m"
#define RED_START "\033[31m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

namespace graph_msf {

class TrajectoryAlignment {
 public:

  template<typename Vec, typename T>
  inline Vec lerp(const Vec& a, const Vec& b, T t) { return a + t*(b - a); }

  // Resample a trajectory to uniform timestamps.
  // Minimal safety fixes:
  // - handle <2 poses
  // - clamp to endpoints to avoid OOB
  // - guard against duplicate timestamps
  Trajectory resample(const Trajectory& src, double t0, double dt, size_t N)
  {
    Trajectory dst;
    const auto& poses = src.poses();
    if (poses.size() < 2 || N == 0) {
      return dst;
    }

    const double t_first = poses.front().time();
    const double t_last  = poses.back().time();

    size_t i = 0;
    for (size_t k = 0; k < N; ++k) {
      double tk = t0 + k * dt;

      if (tk <= t_first) {
        dst.addPose(poses.front().position(), tk);
        continue;
      }
      if (tk >= t_last) {
        dst.addPose(poses.back().position(), tk);
        continue;
      }

      while (i + 1 < poses.size() && poses[i + 1].time() < tk) {
        ++i;
      }
      if (i + 1 >= poses.size()) {
        // Should not happen due to tk < t_last, but keep it safe.
        dst.addPose(poses.back().position(), tk);
        continue;
      }

      const auto& p0 = poses[i];
      const auto& p1 = poses[i + 1];

      const double denom = (p1.time() - p0.time());
      double u = 0.0;
      if (std::abs(denom) > 1e-12) {
        u = (tk - p0.time()) / denom;
      }
      // Clamp interpolation to segment endpoints (handles small numerical drift)
      if (u < 0.0) u = 0.0;
      if (u > 1.0) u = 1.0;

      dst.addPose(lerp(p0.position(), p1.position(), u), tk);
    }
    return dst;
  }

  TrajectoryAlignment();

  // Methods
  void addSe3Position(Eigen::Vector3d position, double time);
  void addR3Position(Eigen::Vector3d position, double time);
  void addR3PositionWithStdDev(Eigen::Vector3d position, double time, Eigen::Vector3d stdDev);
  bool alignTrajectories(double& yaw, Eigen::Isometry3d& returnTransform);
  
  // Setters
  void setR3Rate(const double r3Rate) { r3Rate_ = r3Rate; }
  void setSe3Rate(const double se3Rate) { se3Rate_ = se3Rate; }
  void setMinDistanceHeadingInit(const double minDistanceHeadingInit) { minDistanceHeadingInit_ = minDistanceHeadingInit; }
  void setNoMovementDistance(const double noMovementDistance) { noMovementDistance_ = noMovementDistance; }
  void setNoMovementTime(const double noMovementTime) { noMovementTime_ = noMovementTime; }

  // Getters
  std::vector<std::pair<double, Eigen::Vector3d>> getSe3Trajectory();
  std::vector<std::pair<double, Eigen::Vector3d>> getR3Trajectory();

 private:
  // Member methods
  bool associateTrajectories(const Trajectory& trajectoryA, const Trajectory& trajectoryB, Trajectory& newTrajectoryA, Trajectory& newTrajectoryB);
  bool hasMinimumSpatialSpread(const std::vector<Pose>& poses, double kMinSpread);
  bool trajectoryAlignment(Trajectory& trajectoryA, Trajectory& trajectoryB, Eigen::Isometry3d& returnTransform);
  bool trajectoryAlignmentRobust(
          const Trajectory&  trajectoryA,
          const Trajectory&  trajectoryB,
          Eigen::Isometry3d& returnTransform,
          double             inlierThreshold,
          double             ransacConfidence,
          std::size_t        maxIterations,
          bool               withScaling);

  // Member variables
  Trajectory r3Trajectory_;
  Trajectory se3Trajectory_;

  // Reference Parameters
  double r3Rate_{20.0};
  double se3Rate_{10.0};
  double minDistanceHeadingInit_{3.0};
  double noMovementDistance_{1.0};
  double noMovementTime_{3.0};
  bool firstAlignmentTryFlag_{true};

  // Mutex for adding new measurements
  std::mutex alignmentMutex;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_TrajectoryAlignment_H
