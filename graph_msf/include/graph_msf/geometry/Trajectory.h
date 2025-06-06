#ifndef GRAPH_MSF_TRAJECTORY_H
#define GRAPH_MSF_TRAJECTORY_H

#include "graph_msf/geometry/Pose.h"

namespace graph_msf {

/** \brief Class for holding a trajectory.
 *
 * This class provides buffered access to poses to the represented trajectory.
 */
class Trajectory {
 public:
  Trajectory() : _poses(std::vector<Pose>()) {}

  Trajectory(const Trajectory& other) : _poses(other._poses) {}

  void addPose(Eigen::Vector3d position, double time) {
    Pose newPose(position, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), time);
    _poses.push_back(newPose);
  }

  void addPose(Pose pose) { _poses.push_back(pose); }

  double distance() {
    double distance = 0.0;
    if (_poses.size() < 2) return distance;
    Eigen::Vector3d lastPose = _poses.front().position();
    for (auto poseIt = std::next(_poses.begin()); poseIt != _poses.end(); ++poseIt) {
      distance += ((poseIt->position() - lastPose).norm());
      lastPose = poseIt->position();
    }

    return distance;
  }

  bool isStanding(const double rate, const double seconds, const double& noMovementThreshold) {
    if (_poses.size() < rate * seconds) {
      return false;
    }

    double distance = 0.0;
    Eigen::Vector3d lastPose = (_poses.end() - rate * seconds)->position();
    for (auto poseIt = _poses.end() - rate * seconds + 1; poseIt != _poses.end(); ++poseIt) {
      distance += ((poseIt->position() - lastPose).norm());
      lastPose = poseIt->position();
    }
    return distance < noMovementThreshold;
  }

  std::vector<Pose> poses() const { return _poses; }

  // Cut the trajectory to the time window
  void cutTrajectory(const double& startTime, const double& endTime) {
    // Find the first pose that is within the time window
    auto firstPose = std::find_if(_poses.begin(), _poses.end(), [startTime](const Pose& pose) { return pose.time() >= startTime; });
    // Find the last pose that is within the time window
    auto lastPose = std::find_if(_poses.rbegin(), _poses.rend(), [endTime](const Pose& pose) { return pose.time() <= endTime; });
    // Erase the poses outside the time window
    _poses.erase(lastPose.base(), _poses.end());
    _poses.erase(_poses.begin(), firstPose);
  }

  // Downsample the trajectory by a factor
  void downsample(const int& factor) {
    std::vector<Pose> downsampledPoses;
    for (int i = 0; i < _poses.size(); i += factor) {
      downsampledPoses.push_back(_poses[i]);
    }
    _poses = downsampledPoses;
  }

 private:
  std::vector<Pose> _poses;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_TRAJECTORY_H
