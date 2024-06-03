/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef INTERFACE_NavState_H
#define INTERFACE_NavState_H

// C++
#include <mutex>

// Eigen
#include <utility>

// Workspace
#include "graph_msf/core/GraphState.hpp"

namespace graph_msf {

// NavStae ---------------------------------------------------------------------
class NavState {
 public:
  // Default Constructor
  NavState() = default;

  // Copy Constructor
  NavState(const NavState& navState) = default;

  // Regular Constructor
  NavState(const Eigen::Isometry3d& T_W_Ik, Eigen::Vector3d I_v_W_I, Eigen::Vector3d I_w_W_I, const double timeK)
      : T_W_Ik_(T_W_Ik), I_v_W_I_(std::move(I_v_W_I)), I_w_W_I_(std::move(I_w_W_I)), timeK_(timeK) {}

  // Getters
  const Eigen::Isometry3d& getT_W_Ik() const { return T_W_Ik_; }
  const Eigen::Vector3d& getI_v_W_I() const { return I_v_W_I_; }
  const Eigen::Vector3d& getI_w_W_I() const { return I_w_W_I_; }
  double getTimeK() const { return timeK_; }

 protected:
  // Updates --> Keep all measurements consistent
  void updateInWorld(const Eigen::Isometry3d& T_W_Ik, const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I, double timeK);
  void updateYawInWorld(double yaw_W_Ik);
  void updatePositionInWorld(const Eigen::Vector3d& W_t_W_Ik);

  // General Update
  virtual void updateLatestMeasurementTimestamp(double timeK);

 protected:
  // Transformations
  Eigen::Isometry3d T_W_Ik_ = Eigen::Isometry3d::Identity();
  // Corrected Velocities
  Eigen::Vector3d I_v_W_I_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d I_w_W_I_ = Eigen::Vector3d::Zero();
  // Time
  double timeK_;
};

// SafeNavState ----------------------------------------------------------------
class SafeIntegratedNavState : public NavState {
 public:
  // Default Constructor
  SafeIntegratedNavState() = default;

  // Copy Constructor
  SafeIntegratedNavState(const SafeIntegratedNavState& safeIntegratedNavState) : NavState(safeIntegratedNavState) {
    T_W_O_ = safeIntegratedNavState.T_W_O_;
    T_O_Ik_gravityAligned_ = safeIntegratedNavState.T_O_Ik_gravityAligned_;
    relocalizedWorldToOdom_ = safeIntegratedNavState.relocalizedWorldToOdom_;
  }

  // Regular Constructor
  SafeIntegratedNavState(const Eigen::Isometry3d& T_W_O, const Eigen::Isometry3d& T_O_Ik_gravityAligned, const Eigen::Vector3d& I_v_W_I,
                         const Eigen::Vector3d& I_w_W_I, const double timeK, const bool odomNotJump)
      : NavState(T_W_O * T_O_Ik_gravityAligned, I_v_W_I, I_w_W_I, timeK),
        T_W_O_(T_W_O),
        T_O_Ik_gravityAligned_(T_O_Ik_gravityAligned),
        relocalizedWorldToOdom_(odomNotJump) {}

  SafeIntegratedNavState(const Eigen::Isometry3d& T_W_Ik, const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I,
                         const double timeK)
      : NavState(T_W_Ik, I_v_W_I, I_w_W_I, timeK), T_O_Ik_gravityAligned_(T_W_Ik), T_W_O_(Eigen::Isometry3d::Identity()) {}

  // Stters/updaters
  void update(const Eigen::Isometry3d& T_W_O, const Eigen::Isometry3d& T_O_Ik_gravityAligned, const Eigen::Vector3d& I_v_W_I,
              const Eigen::Vector3d& I_w_W_I, double timeK, bool odomNotJump);
  void updateInWorld(const Eigen::Isometry3d& T_W_Ik, const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I, const double timeK,
                     bool odomNotJump);
  void updateYawInWorld(double yaw_W_Ik, bool odomNotJump);
  void updatePositionInWorld(const Eigen::Vector3d& W_t_W_Ik, bool odomNotJump);

  // General Update
  void updateLatestMeasurementTimestamp(const double timeK) override;

  // Getters
  const Eigen::Isometry3d& getT_W_O() const { return T_W_O_; }

  const Eigen::Isometry3d& getT_O_Ik_gravityAligned() const { return T_O_Ik_gravityAligned_; }

  bool isrelocalizedWorldToOdom() const { return relocalizedWorldToOdom_; }

 private:
  // Transformations
  Eigen::Isometry3d T_W_O_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_O_Ik_gravityAligned_ = Eigen::Isometry3d::Identity();
  // Bool
  bool relocalizedWorldToOdom_;
  // Mutex
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
  SafeNavStateWithCovarianceAndBias(const Eigen::Isometry3d& T_W_Ik, const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I,
                                    const double timeK, const Eigen::Matrix<double, 6, 6>& poseCovariance,
                                    const Eigen::Matrix<double, 3, 3>& velocityCovariance, Eigen::Vector3d accelerometerBias,
                                    Eigen::Vector3d gyroscopeBias, const TransformsDictionary<Eigen::Isometry3d>& fixedFrameTransforms,
                                    const TransformsDictionary<Eigen::Matrix<double, 6, 6>>& fixedFrameTransformsCovariance)
      : NavState(T_W_Ik, I_v_W_I, I_w_W_I, timeK),
        poseCovariance_(poseCovariance),
        velocityCovariance_(velocityCovariance),
        accelerometerBias_(std::move(accelerometerBias)),
        gyroscopeBias_(std::move(gyroscopeBias)),
        fixedFrameTransforms_(fixedFrameTransforms),
        fixedFrameTransformsCovariance_(fixedFrameTransformsCovariance) {}

  // From Optimized State
  explicit SafeNavStateWithCovarianceAndBias(const GraphState& graphState)
      : SafeNavStateWithCovarianceAndBias(Eigen::Isometry3d(graphState.navState().pose().matrix()), graphState.navState().velocity(),
                                          graphState.angularVelocityCorrected(), graphState.ts(), graphState.poseCovariance(),
                                          graphState.velocityCovariance(), graphState.imuBias().accelerometer(),
                                          graphState.imuBias().gyroscope(), graphState.fixedFrameTransforms(),
                                          graphState.fixedFrameTransformsCovariance()) {}

  // Getters
  const Eigen::Matrix<double, 6, 6>& getPoseCovariance() const { return poseCovariance_; }  // yaw, pitch, roll, x, y, z
  const Eigen::Matrix<double, 3, 3>& getVelocityCovariance() const { return velocityCovariance_; }
  const Eigen::Vector3d& getAccelerometerBias() const { return accelerometerBias_; }
  const Eigen::Vector3d& getGyroscopeBias() const { return gyroscopeBias_; }
  const TransformsDictionary<Eigen::Isometry3d>& getFixedFrameTransforms() const { return fixedFrameTransforms_; }
  const TransformsDictionary<Eigen::Matrix<double, 6, 6>>& getFixedFrameTransformsCovariance() const {
    return fixedFrameTransformsCovariance_;
  }

 private:
  // Covariance
  Eigen::Matrix<double, 6, 6> poseCovariance_;  // x,y,z,roll,pitch,yaw
  Eigen::Matrix<double, 3, 3> velocityCovariance_;
  // Bias
  Eigen::Vector3d accelerometerBias_;
  Eigen::Vector3d gyroscopeBias_;
  // Fixed Frame Transforms
  graph_msf::TransformsDictionary<Eigen::Isometry3d> fixedFrameTransforms_;                      // fixed frame transforms
  graph_msf::TransformsDictionary<Eigen::Matrix<double, 6, 6>> fixedFrameTransformsCovariance_;  // fixed frame transforms covariance
};

}  // namespace graph_msf

#endif  // INTERFACE_NavState_H
