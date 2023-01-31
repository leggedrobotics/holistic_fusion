/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef INTERFACE_NavState_H
#define INTERFACE_NavState_H

#include <Eigen/Eigen>

namespace graph_msf {

// Interface Prediction
struct NavState {
  // Copy Constructor
  NavState(const NavState& navState)
      : T_W_M(navState.T_W_M),
        T_M_O(navState.T_M_O),
        T_O_Ik(navState.T_O_Ik),
        I_v_W_I(navState.I_v_W_I),
        I_w_W_I(navState.I_w_W_I),
        timeK(navState.timeK),
        relocalizeWorldToMap(navState.relocalizeWorldToMap) {}
  // Regular Constructor
  NavState(const Eigen::Isometry3d& T_W_M, const Eigen::Isometry3d T_M_O, const Eigen::Isometry3d& T_O_Ik, const Eigen::Vector3d& I_v_W_I,
           const Eigen::Vector3d& I_w_W_I, const double timeK, const bool relocalizeWorldToMap = false)
      : T_W_M(std::move(T_W_M)),
        T_M_O(std::move(T_M_O)),
        T_O_Ik(std::move(T_O_Ik)),
        I_v_W_I(std::move(I_v_W_I)),
        I_w_W_I(std::move(I_w_W_I)),
        timeK(timeK),
        relocalizeWorldToMap(relocalizeWorldToMap) {}

  // Transformations
  const Eigen::Isometry3d T_W_M;
  const Eigen::Isometry3d T_M_O;
  const Eigen::Isometry3d T_O_Ik;
  // Corrected Velocities
  const Eigen::Vector3d I_v_W_I;
  const Eigen::Vector3d I_w_W_I;
  // Time
  double timeK;
  // Bool
  bool relocalizeWorldToMap;
};

struct NavStateWithCovariance : public NavState {
  NavStateWithCovariance(const Eigen::Isometry3d& T_W_M, const Eigen::Isometry3d& T_M_O, const Eigen::Isometry3d& T_O_Ik,
                         const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I, const double timeK,
                         const Eigen::Matrix<double, 6, 6>& poseCovariance, const Eigen::Matrix<double, 3, 3>& velocityCovariance)
      : NavState(T_W_M, T_M_O, T_O_Ik, I_v_W_I, I_w_W_I, timeK), poseCovariance(poseCovariance), velocityCovariance(velocityCovariance) {}

  // Covariance
  const Eigen::Matrix<double, 6, 6> poseCovariance;
  const Eigen::Matrix<double, 3, 3> velocityCovariance;
};

}  // namespace graph_msf

#endif  // INTERFACE_NavState_H
