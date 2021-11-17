//
// Created by nubertj on 30.06.21.
//

#include "fg_filtering/StaticTransforms.h"

namespace compslam_se {

StaticTransforms::StaticTransforms(ros::NodeHandle& privateNode) {
  ROS_INFO("Static Transforms container initializing...");
  std::string urdfDescriptionName;
  if (privateNode.getParam("launch/description_name", urdfDescriptionName)) {
    ROS_INFO_STREAM("FactorGraphFiltering - URDF-Description-Name: " << urdfDescriptionName);
    privateNode.getParam(std::string("/") + urdfDescriptionName, urdfDescription_);
    // urdfDescription_ = sParam;
    if (urdfDescription_.empty()) {
      ROS_ERROR("Could not load description!");
      return;
    }
  } else {
    ROS_ERROR("FactorGraphFiltering - urdf description not set.");
    throw std::runtime_error("Robot description must be provided in rosparams.");
  }
  // load excavator model from URDF
  double timeStep;
  urdfModel_.initParam(urdfDescriptionName);
  excavatorModelPtr_ = std::make_unique<excavator_model::ExcavatorModel>(timeStep);
  excavatorModelPtr_->initModelFromUrdf(urdfDescription_.c_str());
}

void StaticTransforms::findTransformations() {
  ROS_WARN("Looking up transformations in URDF model...");
  // Cabin IMU
  const unsigned int imuCabinBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("IMU_CABIN_link").c_str());
  const unsigned int imuRooftopId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("imu_box_link").c_str());
  if (imuCabinBodyId != std::numeric_limits<unsigned int>::max()) {
    ROS_WARN("Found Body IMU_CABIN.");
    tf_T_Ic_C_ = getTransformFromID(imuCabinBodyId);
    tf_T_C_Ic_ = tf_T_Ic_C_.inverse();
  } else if (imuRooftopId != std::numeric_limits<unsigned int>::max()) {
    ROS_WARN("Did not find Body IMU_CABIN. But found IMU in roofbox.");
    imuCabinFrame_ = imuRooftopFrame_;
    tf_T_Ic_C_ = getTransformFromID(imuRooftopId);
    tf_T_C_Ic_ = tf_T_Ic_C_.inverse();
  } else {
    ROS_ERROR("Neither found Body IMU_CABIN nor rooftop imu!");
    return;
  }
  ROS_WARN_STREAM("IMU frame with respect to cabin frame: t=[" << tf_T_C_Ic_.getOrigin().x() << ", " << tf_T_C_Ic_.getOrigin().y() << ", "
                                                               << tf_T_C_Ic_.getOrigin().z() << "]");
  // LiDAR
  const unsigned int lidarBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("os_lidar").c_str());
  if (lidarBodyId != std::numeric_limits<unsigned int>::max()) {
    tf_T_L_C_ = getTransformFromID(lidarBodyId);
    tf_T_C_L_ = tf_T_L_C_.inverse();
    tf_T_L_Ic_ = tf_T_L_C_ * tf_T_C_Ic_;
    ROS_WARN_STREAM("IMU frame with respect to LiDAR frame: t=[" << tf_T_L_Ic_.getOrigin().x() << ", " << tf_T_L_Ic_.getOrigin().y() << ", "
                                                                 << tf_T_L_Ic_.getOrigin().z() << "]");
  } else {
    ROS_ERROR("Did not find LiDAR!");
    return;
  }
  // Left GNSS
  const unsigned int gnssLeftBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("GNSS_L").c_str());
  if (gnssLeftBodyId != std::numeric_limits<unsigned int>::max()) {
    tf_T_GnssL_C_ = getTransformFromID(gnssLeftBodyId);
    tf_T_Cabin_GnssL_ = tf_T_GnssL_C_.inverse();
    ROS_WARN_STREAM("Left GNSS with respect to Cabin frame: t=[" << tf_T_Cabin_GnssL_.getOrigin().x() << ", "
                                                                 << tf_T_Cabin_GnssL_.getOrigin().y() << ", "
                                                                 << tf_T_Cabin_GnssL_.getOrigin().z() << "]");
  } else {
    ROS_ERROR("Did not find left GNSS!");
    return;
  }
  // Right GNSS
  const unsigned int gnssRightBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("GNSS_R").c_str());
  if (gnssRightBodyId != std::numeric_limits<unsigned int>::max()) {
    tf_T_GnssR_C_ = getTransformFromID(gnssRightBodyId);
    tf_T_C_GnssR_ = tf_T_GnssR_C_.inverse();
    ROS_WARN_STREAM("Right GNSS to Cabin translation: [" << tf_T_C_GnssR_.getOrigin().x() << ", " << tf_T_C_GnssR_.getOrigin().y() << ", "
                                                         << tf_T_C_GnssR_.getOrigin().z() << "]");
  } else {
    ROS_ERROR("Did not find right GNSS!");
    return;
  }
  // Base to cabin
  Eigen::Vector3d B_t_B_C = excavatorModelPtr_->getPositionBodyToBody(
      excavator_model::RD::BodyEnum::BASE, excavator_model::RD::BodyEnum::CABIN, excavator_model::RD::CoordinateFrameEnum::BASE);
  Eigen::Matrix3d R_B_C;
  R_B_C.setIdentity();
  Eigen::Quaterniond q_B_C(R_B_C);
  tf_T_B_C_.setRotation(tf::Quaternion(q_B_C.x(), q_B_C.y(), q_B_C.z(), q_B_C.w()));
  tf_T_B_C_.setOrigin(tf::Vector3(B_t_B_C.x(), B_t_B_C.y(), B_t_B_C.z()));
  tf_T_C_B_ = tf_T_B_C_.inverse();
  // Base to cabin 2
  std::shared_ptr<const urdf::Joint> cabinJoint = urdfModel_.getJoint("J_TURN");
  if (cabinJoint) {
    BC_Z_offset_ = cabinJoint->parent_to_joint_origin_transform.position.z;
  } else {
    throw std::runtime_error("[M545 tf publisher] Did not find cabin turn joint in model.");
  }
  ROS_WARN_STREAM("Offset from method 1: " << tf_T_B_C_.getOrigin().z());
  ROS_WARN_STREAM("Offset from method 2: " << BC_Z_offset_);

  ROS_WARN("...found all transformations.");

  // Base IMU
  const unsigned int imuBaseBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("IMU_base_link").c_str());
  if (imuBaseBodyId != std::numeric_limits<unsigned int>::max()) {
    ROS_WARN("Found Body IMU_BASE.");
    tf_T_Ib_B_ = getTransformFromID(imuBaseBodyId);
    tf_T_B_Ib_ = tf_T_Ib_B_.inverse();
  } else {
    ROS_ERROR("Did not find base imu!");
    return;
  }
  ROS_WARN_STREAM("Base IMU frame with respect to base frame: t=[" << tf_T_B_Ib_.getOrigin().x() << ", " << tf_T_B_Ib_.getOrigin().y()
                                                                   << ", " << tf_T_B_Ib_.getOrigin().z() << "]");
}

tf::Transform StaticTransforms::getTransformFromID(const unsigned int bodyId) {
  tf::Transform tf_T;
  Eigen::Vector3d fixed_t_fixed_body = excavatorModelPtr_->getRbdlModel()
                                           .mFixedBodies[bodyId - excavatorModelPtr_->getRbdlModel().fixed_body_discriminator]
                                           ->mParentTransform.r;
  Eigen::Matrix3d R_body_fixed = excavatorModelPtr_->getRbdlModel()
                                     .mFixedBodies[bodyId - excavatorModelPtr_->getRbdlModel().fixed_body_discriminator]
                                     ->mParentTransform.E;
  Eigen::Vector3d body_t_body_fixed = R_body_fixed * (-fixed_t_fixed_body);
  Eigen::Quaterniond q_body_fixed(R_body_fixed);
  tf_T.setRotation(tf::Quaternion(q_body_fixed.x(), q_body_fixed.y(), q_body_fixed.z(), q_body_fixed.w()));
  tf_T.setOrigin(tf::Vector3(body_t_body_fixed.x(), body_t_body_fixed.y(), body_t_body_fixed.z()));
  return tf_T;
}

}  // namespace compslam_se
