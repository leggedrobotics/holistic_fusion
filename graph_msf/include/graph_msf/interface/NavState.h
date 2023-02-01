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
      : T_W_M_(std::move(T_W_M)),
        T_M_O_(std::move(T_M_O)),
        T_O_Ik_gravityAligned_(std::move(T_O_Ik_gravityAligned)),
        I_v_W_I_(std::move(I_v_W_I)),
        I_w_W_I_(std::move(I_w_W_I)),
        timeK_(timeK),
        relocalizeWorldToMap_(relocalizeWorldToMap) {}

  // Updates
  void update(const Eigen::Isometry3d& T_W_Ik, const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I, const double timeK,
              const bool reLocalizeWorldToMap);
  void updateGlobalYaw(const double yaw_W_Ik, const bool reLocalizeWorldToMap);
  void updateGlobalPosition(const Eigen::Vector3d W_t_W_Ik, const bool reLocalizeWorldToMap);
  void updateTime(const double timeK) { this->timeK_ = timeK; }

  // Calculate pose in world frame
  Eigen::Isometry3d getT_W_Ik() const { return T_W_M_ * T_M_O_ * T_O_Ik_gravityAligned_; }
  Eigen::Isometry3d getT_W_O() const { return T_W_M_ * T_M_O_; }
  // Getters
  const Eigen::Isometry3d& getT_W_M() const { return T_W_M_; }
  const Eigen::Isometry3d& getT_M_O() const { return T_M_O_; }
  const Eigen::Isometry3d& getT_O_Ik_gravityAligned() const { return T_O_Ik_gravityAligned_; }
  const Eigen::Vector3d& getI_v_W_I() const { return I_v_W_I_; }
  const Eigen::Vector3d& getI_w_W_I() const { return I_w_W_I_; }
  double getTimeK() const { return timeK_; }
  bool getRelocalizeWorldToMap() const { return relocalizeWorldToMap_; }

 private:  // Member Variables
  Eigen::Isometry3d T_W_M_;
  Eigen::Isometry3d T_M_O_;
  Eigen::Isometry3d T_O_Ik_gravityAligned_;
  // Corrected Velocities
  Eigen::Vector3d I_v_W_I_;
  Eigen::Vector3d I_w_W_I_;
  // Time
  double timeK_;
  // Bool
  bool relocalizeWorldToMap_;
};

class NavStateWithCovarianceAndBias : public NavState {
 public:
  NavStateWithCovarianceAndBias(const Eigen::Isometry3d& T_W_M, const Eigen::Isometry3d& T_M_O, const Eigen::Isometry3d& T_O_Ik,
                                const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I, const double timeK,
                                const Eigen::Matrix<double, 6, 6>& poseCovariance, const Eigen::Matrix<double, 3, 3>& velocityCovariance,
                                const Eigen::Vector3d& accelerometerBias, const Eigen::Vector3d& gyroscopeBias)
      : NavState(T_W_M, T_M_O, T_O_Ik, I_v_W_I, I_w_W_I, timeK),
        poseCovariance_(poseCovariance),
        velocityCovariance_(velocityCovariance),
        accelerometerBias_(accelerometerBias),
        gyroscopeBias_(gyroscopeBias) {}

  // Getters
  const Eigen::Matrix<double, 6, 6>& getPoseCovariance() const { return poseCovariance_; }
  const Eigen::Matrix<double, 3, 3>& getVelocityCovariance() const { return velocityCovariance_; }
  const Eigen::Vector3d& getAccelerometerBias() const { return accelerometerBias_; }
  const Eigen::Vector3d& getGyroscopeBias() const { return gyroscopeBias_; }

 private:  // Member Variables
  // Covariance
  Eigen::Matrix<double, 6, 6> poseCovariance_;
  Eigen::Matrix<double, 3, 3> velocityCovariance_;

  // Bias
  Eigen::Vector3d accelerometerBias_;
  Eigen::Vector3d gyroscopeBias_;
};

}  // namespace graph_msf

#endif  // INTERFACE_NavState_H
