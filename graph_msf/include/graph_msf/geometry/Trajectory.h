#ifndef GRAPH_MSF_TRAJECTORY_H
#define GRAPH_MSF_TRAJECTORY_H

#include <vector>
#include <algorithm>
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
    Pose newPose(position, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), time);
    _poses.push_back(newPose);
  }

  void addPose(const Pose& pose) {
    _poses.push_back(pose);
  }

  void addPoseWithFilter(const Eigen::Vector3d& position,
                         double                time,
                         const Eigen::Vector3d& covariance,
                         double                timeConstant = 0.5) // [s]
  {
    if (_poses.empty()) {                                // first sample
      _filteredPos   = position;
      _filteredCov   = covariance;
      _poses.emplace_back(position, Eigen::Vector4d(0,0,0,1), time);
      _lastTime      = time;
      return;
    }

    /* --- 1. weights --------------------------------------------------- */
    double dt        = std::max(1e-6, time - _lastTime);   // avoid 0
    double alpha_t   = 1.0 - std::exp(-dt / timeConstant); // smoother for slow dt

    double w_new     = 1.0 / (covariance.array().max(1e-6)).mean();
    double w_old     = 1.0 / (_filteredCov.array().max(1e-6)).mean();
    double alpha_cov = w_new / (w_new + w_old);            // 0-1

    double alpha     = std::clamp(alpha_t * alpha_cov, 0.0, 1.0);

    /* --- 2. exponential update --------------------------------------- */
    _filteredPos = (1.0 - alpha) * _filteredPos + alpha * position;
    _filteredCov = (1.0 - alpha) * _filteredCov + alpha * covariance;

    /* --- 3. store the filtered pose ---------------------------------- */
    _poses.emplace_back(_filteredPos, Eigen::Vector4d(0,0,0,1), time);
    _lastTime = time;
  }

  void clear() {
    _poses.clear();
  }

  // Optionally release heap memory
  void releaseMemory() {
    std::vector<Pose>().swap(_poses);
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

  bool isStanding(double rate, double seconds, double noMovementThreshold) const {
    size_t required_size = static_cast<size_t>(rate * seconds);
    if (_poses.size() < required_size) {
      return false;
    }
    double distance = 0.0;
    auto beginIt = _poses.end() - required_size;
    Eigen::Vector3d lastPose = beginIt->position();
    for (auto poseIt = beginIt + 1; poseIt != _poses.end(); ++poseIt) {
      distance += (poseIt->position() - lastPose).norm();
      lastPose = poseIt->position();
    }
    return distance < noMovementThreshold;
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
  }

 private:
  std::vector<Pose> _poses;

  Eigen::Vector3d _filteredPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d _filteredCov = Eigen::Vector3d::Zero();
  double          _lastTime    = 0.0;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_TRAJECTORY_H
