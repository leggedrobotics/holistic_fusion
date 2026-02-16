#ifndef GRAPH_MSF_TRAJECTORY_H
#define GRAPH_MSF_TRAJECTORY_H

#include <vector>
#include <algorithm>
#include <iterator>
#include <cmath>
#include <Eigen/Core>
#include "graph_msf/geometry/Pose.h"

namespace graph_msf {

/** \brief Class for holding a trajectory.
 *
 * This class provides buffered access to poses to the represented trajectory.
 */
class Trajectory {
 public:
  Trajectory() = default;
  Trajectory(const Trajectory& other) = default;
  Trajectory& operator=(const Trajectory& other) = default;
  Trajectory(Trajectory&& other) noexcept = default;
  Trajectory& operator=(Trajectory&& other) noexcept = default;
  ~Trajectory() = default;

  void addPose(const Eigen::Vector3d& position, double time) {
    _poses.emplace_back(position, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), time);
  }

  void addPose(const Pose& pose) {
    _poses.push_back(pose);
  }

  void addPoseWithFilter(const Eigen::Vector3d& position,
                         double                time,
                         const Eigen::Vector3d& covariance,
                         double                timeConstant = 0.5)
  {
    timeConstant = std::max(1e-6, timeConstant);

    if (_poses.empty()) {
      _filteredPos   = position;
      _filteredCov   = covariance;
      _poses.emplace_back(position, Eigen::Vector4d(0,0,0,1), time);
      _lastTime      = time;
      return;
    }

    double dt        = std::max(1e-6, time - _lastTime);
    double alpha_t   = 1.0 - std::exp(-dt / timeConstant);

    double w_new     = 1.0 / (covariance.array().max(1e-6)).mean();
    double w_old     = 1.0 / (_filteredCov.array().max(1e-6)).mean();
    double alpha_cov = w_new / (w_new + w_old);

    double alpha = alpha_t * alpha_cov;
    if (alpha < 0.0) {
      alpha = 0.0;
    } else if (alpha > 1.0) {
      alpha = 1.0;
    }

    _filteredPos = (1.0 - alpha) * _filteredPos + alpha * position;
    _filteredCov = (1.0 - alpha) * _filteredCov + alpha * covariance;

    _poses.emplace_back(_filteredPos, Eigen::Vector4d(0,0,0,1), time);
    _lastTime = time;
  }

  void clear() {
    _poses.clear();
    resetFilterState_();
  }

  // Optionally release heap memory
  void releaseMemory() {
    std::vector<Pose>().swap(_poses);
    resetFilterState_();
  }

  double distance() const {
    double distance = 0.0;
    if (_poses.size() < 2) return distance;
    Eigen::Vector3d lastPose = _poses.front().position();
    for (auto poseIt = std::next(_poses.begin()); poseIt != _poses.end(); ++poseIt) {
      distance += (poseIt->position() - lastPose).norm();
      lastPose = poseIt->position();
    }
    return distance;
  }

  double displacement() const
    {
      return (_poses.size() < 2)
            ? 0.0
            : (_poses.back().position() - _poses.front().position()).norm();
    }

  bool isStanding(
                  double seconds,
                  double noMovementThreshold) const
  {
    if (_poses.size() < 2) return false;
    if (seconds <= 0.0)   return false;

    // Require non-decreasing time for correctness (caller can sort once upstream).
    // If you cannot guarantee it, consider sorting a copy or using min/max time scans.
    const double t_end = _poses.back().time();
    const double t_start = t_end - seconds;

    // Find first pose within the time window [t_start, t_end]
    auto it0 = std::lower_bound(
        _poses.begin(), _poses.end(), t_start,
        [](const Pose& p, double t) { return p.time() < t; });

    // Not enough samples in the time window
    if (it0 == _poses.end()) return false;
    if (std::distance(it0, _poses.end()) < 2) return false;

    const Eigen::Vector3d p0 = it0->position();
    const Eigen::Vector3d p1 = _poses.back().position();

    // Displacement over the window
    const double disp = (p1 - p0).norm();
    if (disp >= noMovementThreshold) return false;

    // Optional additional guard: max deviation from the start point
    // Helps avoid declaring standing when trajectory "wiggles" but ends near start.
    double maxRadius = 0.0;
    for (auto it = it0; it != _poses.end(); ++it) {
      maxRadius = std::max(maxRadius, (it->position() - p0).norm());
    }

    // You can tune this; using the same threshold is conservative and simple.
    return maxRadius < noMovementThreshold;
  }

  const std::vector<Pose>& poses() const { return _poses; }
  std::vector<Pose>& poses() { return _poses; }

  // Cut the trajectory to the time window [startTime, endTime]
  void cutTrajectory(double startTime, double endTime) {
    // Remove poses before startTime
    auto firstPose = std::find_if(_poses.begin(), _poses.end(),
                                  [startTime](const Pose& pose) { return pose.time() >= startTime; });
    _poses.erase(_poses.begin(), firstPose);

    // Remove poses after endTime
    auto lastPose = std::find_if(_poses.begin(), _poses.end(),
                                 [endTime](const Pose& pose) { return pose.time() > endTime; });
    _poses.erase(lastPose, _poses.end());

    syncFilterStateToBack_();
  }

  // Downsample the trajectory by a factor (keeps 0, factor, 2*factor, ...)
  void downsample(int factor) {
    if (factor <= 1 || _poses.empty()) return;
    std::vector<Pose> downsampledPoses;
    downsampledPoses.reserve((_poses.size() + factor - 1) / factor);
    for (size_t i = 0; i < _poses.size(); i += factor) {
      downsampledPoses.push_back(_poses[i]);
    }
    _poses = std::move(downsampledPoses);

    syncFilterStateToBack_();
  }

 private:
  void resetFilterState_() {
    _filteredPos = Eigen::Vector3d::Zero();
    _filteredCov = Eigen::Vector3d::Zero();
    _lastTime    = 0.0;
  }

  void syncFilterStateToBack_() {
    if (_poses.empty()) {
      resetFilterState_();
      return;
    }
    _filteredPos = _poses.back().position();
    _filteredCov = Eigen::Vector3d::Ones();
    _lastTime    = _poses.back().time();
  }

 private:
  std::vector<Pose> _poses;

  Eigen::Vector3d _filteredPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d _filteredCov = Eigen::Vector3d::Zero();
  double          _lastTime    = 0.0;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_TRAJECTORY_H
