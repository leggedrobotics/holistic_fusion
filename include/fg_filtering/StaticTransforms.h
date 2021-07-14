#ifndef MENZI_SIM_STATICTRANSFORMS_H
#define MENZI_SIM_STATICTRANSFORMS_H

#include "excavator_model/ExcavatorModel.hpp"

namespace fg_filtering {

class StaticTransforms {
 public:
  StaticTransforms(ros::NodeHandle& privateNode) {
    ROS_INFO("Static Transforms container initializing...");
    std::string urdfDescriptionName;
    if (privateNode.getParam("description_name", urdfDescriptionName)) {
      ROS_INFO_STREAM("FactorGraphFiltering - URDF-Description-Name: " << urdfDescriptionName);
      privateNode.getParam(std::string("/") + urdfDescriptionName, urdfDescription_);
      // urdfDescription_ = sParam;
      if (urdfDescription_.empty()) {
        ROS_ERROR("Could not load description!");
        return;
      }
    } else {
      ROS_ERROR("FactorGraphFiltering - urdf description not set.");
      throw runtime_error("Robot description must be provided in rosparams.");
    }
    // load excavator model from URDF
    double timeStep;
    urdfModel_.initParam(urdfDescriptionName);
    excavatorModelPtr_ = std::make_unique<excavator_model::ExcavatorModel>(timeStep);
    excavatorModelPtr_->initModelFromUrdf(urdfDescription_.c_str());
  }

  // Setters
  void setOdomFrame(const std::string& s) { odomFrame_ = s; }

  void setBaseLinkFrame(const std::string& s) { baseLinkFrame_ = s; }

  void setImuCabinFrame(const std::string& s) { imuCabinFrame_ = s; }

  void setImuRooftopFrame(const std::string& s) { imuRooftopFrame_ = s; }

  void setLidarFrame(const std::string& s) { lidarFrame_ = s; }

  void setCabinFrame(const std::string& s) { cabinFrame_ = s; }

  void setLeftGnssFrame(const std::string& s) { leftGnssFrame_ = s; }

  void setRightGnssFrame(const std::string& s) { rightGnssFrame_ = s; }

  // Getters
  /// Frames
  std::string getOdomFrame() { return odomFrame_; }

  std::string getBaseLinkFrame() { return baseLinkFrame_; }

  std::string getImuFrame() { return imuFrame_; }

  std::string getLidarFrame() { return lidarFrame_; }

  std::string getCabinFrame() { return cabinFrame_; }

  std::string getLeftGnssFrame() { return leftGnssFrame_; }

  std::string getRightGnssFrame() { return rightGnssFrame_; }

  /// Transformations
  const tf::Transform& T_LC() { return tf_T_LC_; }

  const tf::Transform& T_CL() { return tf_T_CL_; }

  const tf::Transform& T_LI() { return tf_T_LI_; }

  const tf::Transform& T_IC() { return tf_T_IC_; }

  const tf::Transform& T_CI() { return tf_T_CI_; }

  const tf::Transform& T_CabinGnssL() { return tf_T_CabinGnssL_; }

  const tf::Transform& T_CabinGnssR() { return tf_T_CabinGnssR_; }

  const tf::Transform& T_CB() { return tf_T_CB_; }

  double BC_z_offset() { return BC_Z_offset_; }

  // Functionality
  void findTransformations() {
    ROS_WARN("Looking up transformations in URDF model...");
    // Cabin IMU
    const unsigned int imuBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("IMU_CABIN_link").c_str());
    const unsigned int imuRooftopId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("imu_box_link").c_str());
    if (imuBodyId != std::numeric_limits<unsigned int>::max()) {
      ROS_WARN("Found Body IMU_CABIN.");
      imuFrame_ = imuCabinFrame_;
      tf_T_IC_ = getTransformFromID(imuBodyId);
      tf_T_CI_ = tf_T_IC_.inverse();
    } else if (imuRooftopId != std::numeric_limits<unsigned int>::max()) {
      ROS_WARN("Did not find Body IMU_CABIN. But found IMU in roofbox.");
      imuFrame_ = imuRooftopFrame_;
      tf_T_IC_ = getTransformFromID(imuRooftopId);
      tf_T_CI_ = tf_T_IC_.inverse();
    } else {
      ROS_ERROR("Neither found Body IMU_CABIN nor rooftop imu!");
      return;
    }
    ROS_WARN_STREAM("IMU frame with respect to cabin frame: t=[" << tf_T_CI_.getOrigin().x() << ", " << tf_T_CI_.getOrigin().y() << ", "
                                                                 << tf_T_CI_.getOrigin().z() << "]");
    // LiDAR
    const unsigned int lidarBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("os_lidar").c_str());
    if (lidarBodyId != std::numeric_limits<unsigned int>::max()) {
      tf_T_LC_ = getTransformFromID(lidarBodyId);
      tf_T_CL_ = tf_T_LC_.inverse();
      tf_T_LI_ = tf_T_LC_ * tf_T_CI_;
      ROS_WARN_STREAM("IMU frame with respect to LiDAR frame: t=[" << tf_T_LI_.getOrigin().x() << ", " << tf_T_LI_.getOrigin().y() << ", "
                                                                   << tf_T_LI_.getOrigin().z() << "]");
    } else {
      ROS_ERROR("Did not find LiDAR!");
      return;
    }
    // Left GNSS
    const unsigned int gnssLeftBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("GNSS_L").c_str());
    if (gnssLeftBodyId != std::numeric_limits<unsigned int>::max()) {
      tf_T_GnssLCabin_ = getTransformFromID(gnssLeftBodyId);
      tf_T_CabinGnssL_ = tf_T_GnssLCabin_.inverse();
      ROS_WARN_STREAM("Left GNSS with respect to Cabin frame: t=[" << tf_T_CabinGnssL_.getOrigin().x() << ", "
                                                                   << tf_T_CabinGnssL_.getOrigin().y() << ", "
                                                                   << tf_T_CabinGnssL_.getOrigin().z() << "]");
    } else {
      ROS_ERROR("Did not find left GNSS!");
      return;
    }
    // Right GNSS
    const unsigned int gnssRightBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("GNSS_R").c_str());
    if (gnssRightBodyId != std::numeric_limits<unsigned int>::max()) {
      tf_T_GnssRCabin_ = getTransformFromID(gnssRightBodyId);
      tf_T_CabinGnssR_ = tf_T_GnssRCabin_.inverse();
      ROS_WARN_STREAM("Right GNSS to Cabin translation: [" << tf_T_CabinGnssR_.getOrigin().x() << ", " << tf_T_CabinGnssR_.getOrigin().y()
                                                           << ", " << tf_T_CabinGnssR_.getOrigin().z() << "]");
    } else {
      ROS_ERROR("Did not find right GNSS!");
      return;
    }
    // Base to cabin
    Eigen::Vector3d B_t_BC = excavatorModelPtr_->getPositionBodyToBody(
        excavator_model::RD::BodyEnum::BASE, excavator_model::RD::BodyEnum::CABIN, excavator_model::RD::CoordinateFrameEnum::BASE);
    Eigen::Matrix3d R_BC;
    R_BC.setIdentity();
    Eigen::Quaterniond q_BC(R_BC);
    tf_T_BC_.setRotation(tf::Quaternion(q_BC.x(), q_BC.y(), q_BC.z(), q_BC.w()));
    tf_T_BC_.setOrigin(tf::Vector3(B_t_BC.x(), B_t_BC.y(), B_t_BC.z()));
    tf_T_CB_ = tf_T_BC_.inverse();
    // Base to cabin 2
    std::shared_ptr<const urdf::Joint> cabinJoint = urdfModel_.getJoint("J_TURN");
    if (cabinJoint) {
      BC_Z_offset_ = cabinJoint->parent_to_joint_origin_transform.position.z;
    } else {
      throw runtime_error("[M545 tf publisher] Did not find cabin turn joint in model.");
    }
    ROS_WARN_STREAM("Offset from method 1: " << tf_T_BC_.getOrigin().z());
    ROS_WARN_STREAM("Offset from method 2: " << BC_Z_offset_);

    ROS_WARN("...found all transformations.");
  }

 private:
  // Names
  /// Description
  std::string urdfDescription_;
  /// Frames
  std::string odomFrame_;
  std::string baseLinkFrame_;
  std::string imuCabinFrame_;
  std::string imuRooftopFrame_;
  std::string imuFrame_;
  std::string lidarFrame_;
  std::string cabinFrame_;
  std::string leftGnssFrame_;
  std::string rightGnssFrame_;

  // Robot Models
  urdf::Model urdfModel_;
  std::unique_ptr<excavator_model::ExcavatorModel> excavatorModelPtr_;

  // Transformations
  tf::Transform tf_T_LC_;
  tf::Transform tf_T_CL_;
  tf::Transform tf_T_LI_;
  tf::Transform tf_T_IC_;
  tf::Transform tf_T_CI_;
  tf::Transform tf_T_GnssLCabin_;
  tf::Transform tf_T_CabinGnssL_;
  tf::Transform tf_T_GnssRCabin_;
  tf::Transform tf_T_CabinGnssR_;
  tf::Transform tf_T_BC_;
  tf::Transform tf_T_CB_;
  double BC_Z_offset_;

  // Methods
  tf::Transform getTransformFromID(const unsigned int bodyId) {
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
};

}  // namespace fg_filtering

#endif  // MENZI_SIM_STATICTRANSFORMS_H
