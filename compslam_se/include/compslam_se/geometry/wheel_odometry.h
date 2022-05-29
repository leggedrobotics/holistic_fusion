#ifndef FG_FILTERING_WHEEL_ODOMETRY_H
#define FG_FILTERING_WHEEL_ODOMETRY_H

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace compslam_se {

// For WO factor: Computes Adjoint Map
inline gtsam::Matrix6 adjointMap(const gtsam::Pose3& p)
{
    const gtsam::Matrix3 R = p.rotation().matrix();
    gtsam::Point3 t_ = p.translation();

    gtsam::Matrix3 A = gtsam::skewSymmetric(t_.x(), t_.y(), t_.z()) * R;
    gtsam::Matrix6 adj;
    adj << R, gtsam::Z_3x3, A, R;
    return adj;
}

inline gtsam::Vector1 transform2Yaw(const gtsam::Pose3& wTa, const gtsam::Pose3& wTb, gtsam::OptionalJacobian<1, 6> Hself = boost::none, 
                    gtsam::OptionalJacobian<1, 6> HwTb = boost::none)
{
    // Selection Matrix
    gtsam::Matrix16 sel_mat;
    sel_mat << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    // Jacobians
    if (Hself) {
        *Hself = sel_mat * (-wTb.inverse().AdjointMap() * adjointMap(wTa));
    }

    if (HwTb){
        *HwTb = sel_mat;
    }

    // compute h(x)
    gtsam::Pose3 delta_pose = wTa.inverse() * wTb;
    gtsam::Vector1 delta_plane = gtsam::Vector1(delta_pose.rotation().yaw());

    return delta_plane;
}

}  // end namespace compslam_se

#endif  // FG_FILTERING_WHEEL_ODOMETRY_H