/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef INTERFACE_NavState_H
#define INTERFACE_NavState_H

#include <Eigen/Eigen>
#include <mutex>

namespace graph_msf {

// NavStae ---------------------------------------------------------------------
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
        relocalizedWorldToMap_(relocalizeWorldToMap) {}

  // Getters
  const Eigen::Isometry3d& getT_W_M() const { return T_W_M_; }
  const Eigen::Isometry3d& getT_M_O() const { return T_M_O_; }
  const Eigen::Isometry3d& getT_O_Ik_gravityAligned() const { return T_O_Ik_gravityAligned_; }
  const Eigen::Vector3d& getI_v_W_I() const { return I_v_W_I_; }
  const Eigen::Vector3d& getI_w_W_I() const { return I_w_W_I_; }
  double getTimeK() const { return timeK_; }
  bool isRelocalizedWorldToMap() const { return relocalizedWorldToMap_; }
  // Connection of Frames
  const Eigen::Isometry3d getT_W_Ik() const { return T_W_M_ * T_M_O_ * T_O_Ik_gravityAligned_; }
  const Eigen::Isometry3d getT_M_Ik() const { return T_M_O_ * T_O_Ik_gravityAligned_; }
  const Eigen::Isometry3d getT_W_O() const { return T_W_M_ * T_M_O_; }

 protected:
  // Updates --> Keep all measurements consistent
  // All
  virtual void updateAll(const Eigen::Isometry3d& T_W_M, const Eigen::Isometry3d& T_M_O, const Eigen::Isometry3d& T_O_Ik_gravityAligned,
                         const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I, const double timeK,
                         const bool reLocalizeWorldToMap);
  // Update in World Frame
  virtual void updateInWorld(const Eigen::Isometry3d& T_W_Ik, const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I,
                             const double timeK, const bool reLocalizeWorldToMap);
  virtual void updateYawInWorld(const double yaw_W_Ik, const bool reLocalizeWorldToMap);
  virtual void updatePositionInWorld(const Eigen::Vector3d W_t_W_Ik, const bool reLocalizeWorldToMap);

  // Update in Map Frame
  virtual void updatePoseInMap(const Eigen::Isometry3d& T_M_Ik);
  virtual void updateWorldToMap(const Eigen::Isometry3d& T_W_M);

  // General Update
  virtual void updateLatestMeasurementTimestamp(const double timeK);

 private:  // Member Variables
  Eigen::Isometry3d T_W_M_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_M_O_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_O_Ik_gravityAligned_ = Eigen::Isometry3d::Identity();
  // Corrected Velocities
  Eigen::Vector3d I_v_W_I_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d I_w_W_I_ = Eigen::Vector3d::Zero();
  // Time
  double timeK_;
  // Bool
  bool relocalizedWorldToMap_;
};

// SafeNavState ----------------------------------------------------------------
class SafeNavState : public NavState {
 public:
  // Default Constructor
  SafeNavState() = default;

  // Copy Constructor
  SafeNavState(const SafeNavState& safeNavState) {
    std::lock_guard<std::mutex> stateUpdateLock(stateUpdateMutex_);
    // Copy all member variables
    NavState::updateAll(safeNavState.getT_W_M(), safeNavState.getT_M_O(), safeNavState.getT_O_Ik_gravityAligned(),
                        safeNavState.getI_v_W_I(), safeNavState.getI_w_W_I(), safeNavState.getTimeK(),
                        safeNavState.isRelocalizedWorldToMap());
  }

  // Regular Constructor
  SafeNavState(const Eigen::Isometry3d& T_O_Ik, const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I, const double timeK,
               const bool relocalizeWorldToMap = false)
      : NavState(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), T_O_Ik, I_v_W_I, I_w_W_I, timeK, relocalizeWorldToMap) {}

  // Updates
  // Update in World Frame
  void updateInWorld(const Eigen::Isometry3d& T_W_Ik, const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I, const double timeK,
                     const bool reLocalizeWorldToMap) override;
  void updateYawInWorld(const double yaw_W_Ik, const bool reLocalizeWorldToMap) override;
  void updatePositionInWorld(const Eigen::Vector3d W_t_W_Ik, const bool reLocalizeWorldToMap) override;

  // Update in Map Frame
  void updatePoseInMap(const Eigen::Isometry3d& T_M_Ik) override;
  void updateWorldToMap(const Eigen::Isometry3d& T_W_M) override;

  // General Update
  void updateLatestMeasurementTimestamp(const double timeK) override;

 private:  // Member Variables
  std::mutex stateUpdateMutex_;
};

// SafeNavStateWithCovarianceAndBias -------------------------------------------
class SafeNavStateWithCovarianceAndBias : public NavState {
 public:
  // Default Constructor
  SafeNavStateWithCovarianceAndBias() = default;

  // Copy Constructor
  SafeNavStateWithCovarianceAndBias(const SafeNavStateWithCovarianceAndBias& navState) = default;

  // Regular Constructor
  SafeNavStateWithCovarianceAndBias(const Eigen::Isometry3d& T_W_Ik, const Eigen::Vector3d& I_v_W_I, const double timeK,
                                    const Eigen::Matrix<double, 6, 6>& poseCovariance,
                                    const Eigen::Matrix<double, 3, 3>& velocityCovariance, const Eigen::Vector3d& accelerometerBias,
                                    const Eigen::Vector3d& gyroscopeBias)
      : NavState(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), T_W_Ik, I_v_W_I, Eigen::Vector3d(), timeK),
        poseCovariance_(poseCovariance),
        velocityCovariance_(velocityCovariance),
        accelerometerBias_(accelerometerBias),
        gyroscopeBias_(gyroscopeBias) {}

  // Getters
  const Eigen::Matrix<double, 6, 6>& getPoseCovariance() const { return poseCovariance_; }  // yaw, pitch, roll, x, y, z
  const Eigen::Matrix<double, 3, 3>& getVelocityCovariance() const { return velocityCovariance_; }
  const Eigen::Vector3d& getAccelerometerBias() const { return accelerometerBias_; }
  const Eigen::Vector3d& getGyroscopeBias() const { return gyroscopeBias_; }

 private:  // Member Variables
  // Covariance
  Eigen::Matrix<double, 6, 6> poseCovariance_;  // x,y,z,roll,pitch,yaw
  Eigen::Matrix<double, 3, 3> velocityCovariance_;

  // Bias
  Eigen::Vector3d accelerometerBias_;
  Eigen::Vector3d gyroscopeBias_;
};

}  // namespace graph_msf

#endif  // INTERFACE_NavState_H
