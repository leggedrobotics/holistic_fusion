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
#include "graph_msf/frontend/Terminal.h"

namespace graph_msf {

void NavState::update(const Eigen::Isometry3d& T_W_Ik_new, const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I,
                      const double timeK, const bool reLocalizeWorldToMap) {
  // Always update roll and pitch in odometry frame
  gtsam::Pose3 T_W_Ik_new_gtsam(T_W_Ik_new.matrix());
  double globalRoll = T_W_Ik_new_gtsam.rotation().roll();
  double globalPitch = T_W_Ik_new_gtsam.rotation().pitch();

  gtsam::Pose3 T_W_M_gtsam(T_W_M.matrix());
  std::cout << YELLOW_START << "GMsf" << GREEN_START << " T_W_M roll: " << T_W_M_gtsam.rotation().roll()
            << ", pitch: " << T_W_M_gtsam.rotation().pitch() << COLOR_END << std::endl;

  if (reLocalizeWorldToMap) {
    std::cout << YELLOW_START << "GMsf" << GREEN_START << " Relocalization is needed. Publishing to world->map." << COLOR_END << std::endl;
    // For this computation step assume T_O_Ik ~ T_O_Ikm1 --> there will be no robot translation in the odometry frame
    gtsam::Rot3 R_O_Ik_gravityAligned(T_O_Ik_gravityAligned.rotation().matrix());
    R_O_Ik_gravityAligned = gtsam::Rot3::Ypr(R_O_Ik_gravityAligned.yaw(), globalPitch, globalRoll);
    T_O_Ik_gravityAligned.matrix().block<3, 3>(0, 0) = R_O_Ik_gravityAligned.matrix();
    T_W_M = T_W_Ik_new * T_O_Ik_gravityAligned.inverse() * T_M_O.inverse();
  } else {
    T_O_Ik_gravityAligned = T_M_O.inverse() * T_W_M.inverse() * T_W_Ik_new;
  }
}

void NavState::updateGlobalYaw(const double yaw_W_Ik, const bool reLocalizeWorldToMap) {
  // Use GTSAM for correcto yaw convention
  gtsam::Pose3 T_O_Ik_old(T_O_Ik_gravityAligned.matrix());
  gtsam::Pose3 T_W_Ik_old = gtsam::Pose3(T_W_M * T_M_O * T_O_Ik_old.matrix());
  gtsam::Pose3 T_W_Ik_new =
      gtsam::Pose3(gtsam::Rot3::Ypr(yaw_W_Ik, T_W_Ik_old.rotation().pitch(), T_W_Ik_old.rotation().roll()), T_W_Ik_old.translation());
  if (reLocalizeWorldToMap) {
    T_W_M = ((T_W_Ik_new * T_O_Ik_old.inverse()).matrix() * T_M_O.inverse()).matrix();
  } else {
    T_O_Ik_gravityAligned = T_M_O.inverse() * T_W_M.inverse() * T_W_Ik_new.matrix();
  }
}

void NavState::updateGlobalPosition(const Eigen::Vector3d W_t_W_Ik, const bool reLocalizeWorldToMap) {
  gtsam::Pose3 T_O_Ik_old(T_O_Ik_gravityAligned.matrix());
  gtsam::Pose3 T_W_Ik_old = gtsam::Pose3(T_W_M * T_M_O * T_O_Ik_old.matrix());
  gtsam::Pose3 T_W_Ik_new = gtsam::Pose3(T_W_Ik_old.rotation(), W_t_W_Ik);
  if (reLocalizeWorldToMap) {
    T_W_M = ((T_W_Ik_new * T_O_Ik_old.inverse()).matrix() * T_M_O.inverse()).matrix();
  } else {
    T_O_Ik_gravityAligned = T_M_O.inverse() * T_W_M.inverse() * T_W_Ik_new.matrix();
  }
}
}  // namespace graph_msf