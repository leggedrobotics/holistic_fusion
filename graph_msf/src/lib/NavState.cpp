/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf/interface/NavState.h"

// GTSAM
#include <gtsam/geometry/Pose3.h>

// Project
#include "graph_msf/interface/Terminal.h"

namespace graph_msf {

// NavState -------------------------------------------------------------------
void NavState::updateAll(const Eigen::Isometry3d& T_W_M, const Eigen::Isometry3d& T_M_O, const Eigen::Isometry3d& T_O_Ik_gravityAligned,
                         const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I, const double timeK,
                         const bool reLocalizeWorldToMap) {
  T_W_M_ = T_W_M;
  T_M_O_ = T_M_O;
  T_O_Ik_gravityAligned_ = T_O_Ik_gravityAligned;
  I_v_W_I_ = I_v_W_I;
  I_w_W_I_ = I_w_W_I;
  timeK_ = timeK;
  relocalizedWorldToMap_ = reLocalizeWorldToMap;
}

void NavState::updateInWorld(const Eigen::Isometry3d& T_W_Ik_new, const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I,
                             const double timeK, const bool reLocalizeWorldToMap) {
  // Velocities in body frame
  I_v_W_I_ = I_v_W_I;
  I_w_W_I_ = I_w_W_I;

  // Time
  timeK_ = timeK;

  // Relocalize
  relocalizedWorldToMap_ = reLocalizeWorldToMap;

  // Poses
  // Sanity check --> Make sure that roll and pitch is in odometry frame and not world to odometry frame
  gtsam::Pose3 T_W_O_gtsam(getT_W_O().matrix());
  if (T_W_O_gtsam.rotation().roll() > 1e-2 || T_W_O_gtsam.rotation().pitch() > 1e-2) {
    std::cout << YELLOW_START << "GMsf" << RED_START << " T_W_O roll: " << T_W_O_gtsam.rotation().roll()
              << ", pitch: " << T_W_O_gtsam.rotation().pitch() << ", which is supposed to be close to zero!" << COLOR_END << std::endl;
  }
  // Always update roll and pitch in odometry frame
  gtsam::Pose3 T_W_Ik_new_gtsam(T_W_Ik_new.matrix());
  double globalRoll = T_W_Ik_new_gtsam.rotation().roll();
  double globalPitch = T_W_Ik_new_gtsam.rotation().pitch();
  // Decide on relocalization
  if (reLocalizeWorldToMap) {
    std::cout << YELLOW_START << "GMsf" << GREEN_START << " Relocalization is needed. Publishing to world->map." << COLOR_END << std::endl;
    // For this computation step assume T_O_Ik ~ T_O_Ikm1 --> there will be no robot translation in the odometry frame
    gtsam::Rot3 R_O_Ik_gravityAligned(T_O_Ik_gravityAligned_.rotation().matrix());
    R_O_Ik_gravityAligned = gtsam::Rot3::Ypr(R_O_Ik_gravityAligned.yaw(), globalPitch, globalRoll);
    T_O_Ik_gravityAligned_.matrix().block<3, 3>(0, 0) = R_O_Ik_gravityAligned.matrix();
    T_W_M_ = T_W_Ik_new * T_O_Ik_gravityAligned_.inverse() * T_M_O_.inverse();
  } else {
    T_O_Ik_gravityAligned_ = T_M_O_.inverse() * T_W_M_.inverse() * T_W_Ik_new;
  }
}

void NavState::updateYawInWorld(const double yaw_W_Ik, const bool reLocalizeWorldToMap) {
  // Relocalize
  relocalizedWorldToMap_ = reLocalizeWorldToMap;

  // Use GTSAM for correcto yaw convention
  gtsam::Pose3 T_O_Ik_old(T_O_Ik_gravityAligned_.matrix());
  gtsam::Pose3 T_W_Ik_old = gtsam::Pose3(T_W_M_ * T_M_O_ * T_O_Ik_old.matrix());
  gtsam::Pose3 T_W_Ik_new =
      gtsam::Pose3(gtsam::Rot3::Ypr(yaw_W_Ik, T_W_Ik_old.rotation().pitch(), T_W_Ik_old.rotation().roll()), T_W_Ik_old.translation());
  if (reLocalizeWorldToMap) {
    T_W_M_ = ((T_W_Ik_new * T_O_Ik_old.inverse()).matrix() * T_M_O_.inverse()).matrix();
  } else {
    T_O_Ik_gravityAligned_ = T_M_O_.inverse() * T_W_M_.inverse() * T_W_Ik_new.matrix();
  }
}

void NavState::updatePositionInWorld(const Eigen::Vector3d W_t_W_Ik, const bool reLocalizedWorldToMap) {
  // Relocalize
  relocalizedWorldToMap_ = reLocalizedWorldToMap;

  // Position in world frame needs distribution on poses
  gtsam::Pose3 T_O_Ik_old(T_O_Ik_gravityAligned_.matrix());
  gtsam::Pose3 T_W_Ik_old = gtsam::Pose3(T_W_M_ * T_M_O_ * T_O_Ik_old.matrix());
  gtsam::Pose3 T_W_Ik_new = gtsam::Pose3(T_W_Ik_old.rotation(), W_t_W_Ik);
  if (reLocalizedWorldToMap) {
    T_W_M_ = ((T_W_Ik_new * T_O_Ik_old.inverse()).matrix() * T_M_O_.inverse()).matrix();
  } else {
    T_O_Ik_gravityAligned_ = T_M_O_.inverse() * T_W_M_.inverse() * T_W_Ik_new.matrix();
  }
}

void NavState::updatePoseInMap(const Eigen::Isometry3d& T_M_Ik) {
  const Eigen::Isometry3d T_W_Ik_old = getT_W_Ik();
  T_M_O_ = T_M_Ik * T_O_Ik_gravityAligned_.inverse();
  T_W_M_ = T_W_Ik_old * getT_M_Ik().inverse();
}

void NavState::updateWorldToMap(const Eigen::Isometry3d& T_W_M) {
  const Eigen::Isometry3d T_W_O_old = getT_W_O();
  T_W_M_ = T_W_M;
  T_M_O_ = T_W_M_.inverse() * T_W_O_old;
}

void NavState::updateLatestMeasurementTimestamp(const double timeK) {
  timeK_ = timeK;
}

// SafeNavState -----------------------------------------------------------------------------------------------
void SafeNavState::updateInWorld(const Eigen::Isometry3d& T_W_Ik, const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I,
                                 const double timeK, const bool reLocalizeWorldToMap) {
  // Mutex for safety
  std::lock_guard<std::mutex> updateLock(stateUpdateMutex_);
  // Parent class
  NavState::updateInWorld(T_W_Ik, I_v_W_I, I_w_W_I, timeK, reLocalizeWorldToMap);
}

void SafeNavState::updateYawInWorld(const double yaw_W_Ik, const bool reLocalizeWorldToMap) {
  // Mutex for safety
  std::lock_guard<std::mutex> updateLock(stateUpdateMutex_);
  // Parent class
  NavState::updateYawInWorld(yaw_W_Ik, reLocalizeWorldToMap);
}

void SafeNavState::updatePositionInWorld(const Eigen::Vector3d W_t_W_Ik, const bool reLocalizedWorldToMap) {
  // Mutex for safety
  std::lock_guard<std::mutex> updateLock(stateUpdateMutex_);
  // Parent class
  NavState::updatePositionInWorld(W_t_W_Ik, reLocalizedWorldToMap);
}

void SafeNavState::updatePoseInMap(const Eigen::Isometry3d& T_M_Ik) {
  // Mutex for safety
  std::lock_guard<std::mutex> updateLock(stateUpdateMutex_);
  // Parent class
  NavState::updatePoseInMap(T_M_Ik);
}

void SafeNavState::updateWorldToMap(const Eigen::Isometry3d& T_W_M) {
  // Mutex for safety
  std::lock_guard<std::mutex> updateLock(stateUpdateMutex_);
  // Parent class
  NavState::updateWorldToMap(T_W_M);
}

void SafeNavState::updateLatestMeasurementTimestamp(const double timeK) {
  // Mutex for safety
  std::lock_guard<std::mutex> updateLock(stateUpdateMutex_);
  // Parent class
  NavState::updateLatestMeasurementTimestamp(timeK);
}

}  // namespace graph_msf