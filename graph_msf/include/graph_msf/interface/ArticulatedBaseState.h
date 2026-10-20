#pragma once

#include <cmath>
#include <stdexcept>

#include <gtsam/geometry/Pose3.h>

namespace graph_msf {

/**
 * Derive a chassis pose from the cabin IMU pose and turn angle.
 *
 * The covariance must be the JOINT posterior of this pose and angle at the
 * same graph key. Its explicit block order is [Pose3 local rotation (3),
 * Pose3 local translation (3), turn angle (1)]; do not assume that a GTSAM
 * JointMarginal uses caller-requested key order.
 *
 * Extrinsics are treated as known here. Uncertain mounting or leg-gravity
 * calibration needs additional graph variables/covariance before runtime use.
 */
struct ArticulatedBaseState {
  gtsam::Pose3 poseWorldBase;
  gtsam::Matrix6 covarianceBaseLocal;
  // ROS ordering: world translation xyz, world rotation-vector xyz.
  gtsam::Matrix6 covarianceWorldRos;
  Eigen::Matrix<double, 6, 7> jointToBaseJacobian;
};

inline ArticulatedBaseState deriveArticulatedBaseState(
    const gtsam::Pose3& poseWorldImu, double turnAngle,
    const gtsam::Pose3& poseImuCabin,
    const gtsam::Point3& cabinToBaseTranslation,
    const Eigen::Matrix<double, 7, 7>& jointCovariance) {
  if (!poseWorldImu.matrix().allFinite() ||
      !poseImuCabin.matrix().allFinite() ||
      !cabinToBaseTranslation.allFinite() || !std::isfinite(turnAngle) ||
      !jointCovariance.allFinite() ||
      !jointCovariance.isApprox(jointCovariance.transpose(), 1e-10)) {
    throw std::invalid_argument("Invalid articulated pose or joint covariance");
  }

  const gtsam::Pose3 poseCabinBase(gtsam::Rot3::Yaw(-turnAngle),
                                  cabinToBaseTranslation);
  const gtsam::Pose3 poseImuBase = poseImuCabin.compose(poseCabinBase);
  ArticulatedBaseState result;
  result.poseWorldBase = poseWorldImu.compose(poseImuBase);
  result.jointToBaseJacobian.setZero();
  result.jointToBaseJacobian.leftCols<6>() = poseImuBase.inverse().AdjointMap();
  // CABIN->BASE translation is fixed in CABIN; q affects only its rotation.
  result.jointToBaseJacobian(2, 6) = -1.0;
  result.covarianceBaseLocal =
      result.jointToBaseJacobian * jointCovariance *
      result.jointToBaseJacobian.transpose();
  result.covarianceBaseLocal =
      (0.5 * (result.covarianceBaseLocal +
              result.covarianceBaseLocal.transpose())).eval();

  gtsam::Matrix6 localToWorldRos = gtsam::Matrix6::Zero();
  const gtsam::Matrix3 rotationWorldBase = result.poseWorldBase.rotation().matrix();
  localToWorldRos.block<3, 3>(0, 3) = rotationWorldBase;
  localToWorldRos.block<3, 3>(3, 0) = rotationWorldBase;
  result.covarianceWorldRos =
      localToWorldRos * result.covarianceBaseLocal * localToWorldRos.transpose();
  return result;
}

}  // namespace graph_msf
