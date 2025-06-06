/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_STATE_HPP_
#define GRAPH_STATE_HPP_

// C++
#include <mutex>

// GTSAM
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

// Workspace
#include "graph_msf/core/TransformsDictionary.h"

namespace graph_msf {

// Class defining Robot State
class GraphState {
 public:
  GraphState()
      : referenceFrameTransforms_(Eigen::Isometry3d::Identity()),
        referenceFrameTransformsCovariance_(Eigen::Matrix<double, 6, 6>::Zero()),
        landmarkTransforms_(Eigen::Isometry3d::Identity()),
        landmarkTransformsCovariance_(Eigen::Matrix<double, 6, 6>::Zero()){};  // Constructor
  ~GraphState(){};                                                             // Destructor

  // Accessors
  bool isOptimized() const { return isOptimizedStatus_; }
  gtsam::Key key() const { return key_; }
  double ts() const { return ts_; }
  const gtsam::NavState& navState() const { return navState_; }
  const gtsam::Vector3& angularVelocityCorrected() const { return correctedAngularVelocity_; }
  const gtsam::imuBias::ConstantBias& imuBias() const { return imuBias_; }
  const auto& referenceFrameTransforms() const { return referenceFrameTransforms_; }
  const auto& referenceFrameTransformsCovariance() const { return referenceFrameTransformsCovariance_; }
  const auto& landmarkTransforms() const { return landmarkTransforms_; }
  const auto& landmarkTransformsCovariance() const { return landmarkTransformsCovariance_; }
  const gtsam::Matrix66& poseCovariance() const { return poseCovariance_; }
  const gtsam::Matrix33& velocityCovariance() const { return velocityCovariance_; }

  // Status
  void setIsOptimized() { isOptimizedStatus_ = true; }

  // Update state Graph Key and Timestamp
  void updateKeyAndTimestamp(const gtsam::Key key, const double ts) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    key_ = key;
    ts_ = ts;
  }

  // Update state Graph Key, Timestamp and NavState(Pose+Velocity)
  void updateNavState(const gtsam::Key key, const double ts, const gtsam::NavState& navState,
                      const gtsam::Vector3& correctedAngularVelocity) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    key_ = key;
    ts_ = ts;
    navState_ = navState;
    correctedAngularVelocity_ = correctedAngularVelocity;
  }

  // Update state Graph Key, Timestamp and IMU Bias estimate
  void updateImuBias(const gtsam::Key key, const double ts, const gtsam::imuBias::ConstantBias& imuBias) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    key_ = key;
    ts_ = ts;
    imuBias_ = imuBias;
  }

  // Update state Graph Key, Timestamp, NavState(Pose+Velocity) and IMU Bias estimate
  void updateNavStateAndBias(const gtsam::Key key, const double ts, const gtsam::NavState& navState,
                             const gtsam::Vector3& correctedAngularVelocity, const gtsam::imuBias::ConstantBias& imuBias) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    key_ = key;
    ts_ = ts;
    navState_ = navState;
    correctedAngularVelocity_ = correctedAngularVelocity;
    imuBias_ = imuBias;
  }

  void updateReferenceFrameTransforms(const graph_msf::TransformsDictionary<Eigen::Isometry3d>& fixedFrameTransforms) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    referenceFrameTransforms_ = fixedFrameTransforms;
  }

  void updateReferenceFrameTransformsCovariance(
      const graph_msf::TransformsDictionary<Eigen::Matrix<double, 6, 6>>& fixedFrameTransformsCovariance) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    referenceFrameTransformsCovariance_ = fixedFrameTransformsCovariance;
  }

  void updateLandmarkTransforms(const graph_msf::TransformsDictionary<Eigen::Isometry3d>& landmarkTransforms) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    landmarkTransforms_ = landmarkTransforms;
  }

  void updateLandmarkTransformsCovariance(
      const graph_msf::TransformsDictionary<Eigen::Matrix<double, 6, 6>>& landmarkTransformsCovariance) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    landmarkTransformsCovariance_ = landmarkTransformsCovariance;
  }

  void updateCovariances(const gtsam::Matrix66& poseCovariance, const gtsam::Matrix33& velocityCovariance) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    poseCovariance_ = poseCovariance;
    velocityCovariance_ = velocityCovariance;
  }

  // Overload assignment operator
  GraphState& operator=(GraphState& other) {
    this->key_ = other.key_;
    this->ts_ = other.ts_;
    this->navState_ = other.navState_;
    this->imuBias_ = other.imuBias_;
    return *this;
  }

 private:
  // Status
  bool isOptimizedStatus_ = false;
  // Keys and Times
  gtsam::Key key_ = 0;  // key
  double ts_ = 0.0;     // timestamp
  // Objects
  gtsam::NavState navState_;                  // pose, velocity
  Eigen::Vector3d correctedAngularVelocity_;  // angular velocity
  gtsam::imuBias::ConstantBias imuBias_;      // imu bias
  // Transformations
  graph_msf::TransformsDictionary<Eigen::Isometry3d> referenceFrameTransforms_;  // reference frame transforms
  graph_msf::TransformsDictionary<Eigen::Matrix<double, 6, 6>>
      referenceFrameTransformsCovariance_;                                                     // reference frame transforms covariance
  graph_msf::TransformsDictionary<Eigen::Isometry3d> landmarkTransforms_;                      // landmark transforms
  graph_msf::TransformsDictionary<Eigen::Matrix<double, 6, 6>> landmarkTransformsCovariance_;  // landmark transforms covariance
  // Covariances
  gtsam::Matrix66 poseCovariance_;      // pose covariance
  gtsam::Matrix33 velocityCovariance_;  // velocity covariance
  // Mutex
  std::mutex stateMutex_;
};

}  // namespace graph_msf

#endif  // GRAPH_STATE_HPP_