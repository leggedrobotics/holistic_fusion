/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef INTERFACE_NavState_H
#define INTERFACE_NavState_H

#include <Eigen/Eigen>

namespace graph_msf {

// Interface Prediction
class NavState {
 public:
  // Default Constructor
  NavState() = default;

  // Copy Constructor
  NavState(const NavState& navState) = default;

  // Regular Constructor
  NavState(const Eigen::Isometry3d& T_W_M, const Eigen::Isometry3d T_M_O, const Eigen::Isometry3d& T_O_Ik_gravityAligned,
           const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I, const double timeK, const bool relocalizeWorldToMap = false)
      : T_W_M(std::move(T_W_M)),
        T_M_O(std::move(T_M_O)),
        T_O_Ik_gravityAligned(std::move(T_O_Ik_gravityAligned)),
        I_v_W_I(std::move(I_v_W_I)),
        I_w_W_I(std::move(I_w_W_I)),
        timeK(timeK),
        relocalizeWorldToMap(relocalizeWorldToMap) {}

  // Updates
  void update(const Eigen::Isometry3d& T_W_Ik, const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I, const double timeK,
              const bool reLocalizeWorldToMap);
  void updateGlobalYaw(const double yaw_W_Ik, const bool reLocalizeWorldToMap);
  void updateGlobalPosition(const Eigen::Vector3d W_t_W_Ik, const bool reLocalizeWorldToMap);
  void updateTime(const double timeK) { this->timeK = timeK; }

  // Calculate pose in world frame
  Eigen::Isometry3d getT_W_Ik() const { return T_W_M * T_M_O * T_O_Ik_gravityAligned; }

  // Getters
  const Eigen::Isometry3d& getT_W_M() const { return T_W_M; }
  const Eigen::Isometry3d& getT_M_O() const { return T_M_O; }
  const Eigen::Isometry3d& getT_O_Ik_gravityAligned() const { return T_O_Ik_gravityAligned; }
  const Eigen::Vector3d& getI_v_W_I() const { return I_v_W_I; }
  const Eigen::Vector3d& getI_w_W_I() const { return I_w_W_I; }
  double getTimeK() const { return timeK; }
  bool getRelocalizeWorldToMap() const { return relocalizeWorldToMap; }

 private:
  // Member Variables
  Eigen::Isometry3d T_W_M;
  Eigen::Isometry3d T_M_O;
  Eigen::Isometry3d T_O_Ik_gravityAligned;
  // Corrected Velocities
  Eigen::Vector3d I_v_W_I;
  Eigen::Vector3d I_w_W_I;
  // Time
  double timeK;
  // Bool
  bool relocalizeWorldToMap;
};

class NavStateWithCovarianceAndBias : public NavState {
  NavStateWithCovarianceAndBias(const Eigen::Isometry3d& T_W_M, const Eigen::Isometry3d& T_M_O, const Eigen::Isometry3d& T_O_Ik,
                                const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I, const double timeK,
                                const Eigen::Matrix<double, 6, 6>& poseCovariance, const Eigen::Matrix<double, 3, 3>& velocityCovariance,
                                const Eigen::Vector3d& accelerometerBias, const Eigen::Vector3d& gyroscopeBias)
      : NavState(T_W_M, T_M_O, T_O_Ik, I_v_W_I, I_w_W_I, timeK),
        poseCovariance(poseCovariance),
        velocityCovariance(velocityCovariance),
        accelerometerBias(accelerometerBias),
        gyroscopeBias(gyroscopeBias) {}

  // Covariance
  Eigen::Matrix<double, 6, 6> poseCovariance;
  Eigen::Matrix<double, 3, 3> velocityCovariance;

  // Bias
  Eigen::Vector3d accelerometerBias;
  Eigen::Vector3d gyroscopeBias;
};

}  // namespace graph_msf

#endif  // INTERFACE_NavState_H
