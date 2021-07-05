#ifndef MENZI_SIM_STATICTRANSFORMS_H
#define MENZI_SIM_STATICTRANSFORMS_H

#include "excavator_model/ExcavatorModel.hpp"

namespace fg_filtering {

class StaticTransforms {
public:
  StaticTransforms(ros::NodeHandle &privateNode) {
    ROS_INFO("Static Transforms container initializing...");
    std::string sParam;
    if (privateNode.getParam("description_name", sParam)) {
      ROS_INFO_STREAM("FactorGraphFiltering - URDF-Description-Name: " << sParam);
      privateNode.getParam(std::string("/") + sParam, urdfDescription_);
      // urdfDescription_ = sParam;
      if (urdfDescription_.empty()) {
        ROS_ERROR("Could not load description!");
        return;
      }
    } else {
      ROS_ERROR("FactorGraphFiltering - urdf description not set.");
      return;
    }
    // load excavator model from URDF
    double timeStep;
    excavatorModelPtr_ = std::make_unique<excavator_model::ExcavatorModel>(timeStep);
    excavatorModelPtr_->initModelFromUrdf(urdfDescription_.c_str());
  }

  // Setters
  void setBaseLinkFrame(const std::string &s) { baseLinkFrame_ = s; }

  void setImuFrame(const std::string &s) { imuFrame_ = s; }

  void setLidarFrame(const std::string &s) { lidarFrame_ = s; }

  void setCabinFrame(const std::string &s) { cabinFrame_ = s; }

  void setGnssLeftFrame(const std::string &s) { gnssLeftFrame_ = s; }

  void setGnssRightFrame(const std::string &s) { gnssRightFrame_ = s; }

  // Getters
  /// Frames
  std::string getBaseLinkFrame() { return baseLinkFrame_; }

  std::string getImuFrame() { return imuFrame_; }

  std::string getLidarFrame() { return lidarFrame_; }

  std::string getCabinFrame() { return cabinFrame_; }

  std::string getGnssLeftFrame() { return gnssLeftFrame_; }

  std::string getGnssRightFrame() { return gnssRightFrame_; }

  /// Transformations
  const tf::Transform &T_LC() { return tf_T_LC_; }

  const tf::Transform &T_CL() { return tf_T_CL_; }

  const tf::Transform &T_LI() { return tf_T_LI_; }

  const tf::Transform &T_IC() { return tf_T_IC_; }

  const tf::Transform &T_CI() { return tf_T_CI_; }

  const tf::Transform &T_CabinGnssL() { return tf_T_CabinGnssL_; }

  const tf::Transform &T_CabinGnssR() { return tf_T_CabinGnssR_; }

  const tf::Transform &T_CB() { return tf_T_CB_; }

  // Functionality
  void findTransformations() {
    ROS_WARN("Looking up transformations in URDF model...");
    // Cabin IMU
    const unsigned int imuBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("IMU_CABIN_link").c_str());
    if (imuBodyId != std::numeric_limits<unsigned int>::max()) {
      tf_T_CI_ = getTransformFromID(imuBodyId);
      tf_T_IC_ = tf_T_CI_.inverse();
      ROS_WARN_STREAM("IMU to Cabin translation: [" << tf_T_CI_.getOrigin().x() << ", " << tf_T_CI_.getOrigin().y() << ", "
                                                    << tf_T_CI_.getOrigin().z() << "]");
    } else {
      ROS_ERROR("Did not find Cabin IMU!");
      return;
    }
    // LiDAR
    const unsigned int lidarBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("os_sensor").c_str());
    if (lidarBodyId != std::numeric_limits<unsigned int>::max()) {
      tf_T_CL_ = getTransformFromID(lidarBodyId);
      tf_T_LC_ = tf_T_CL_.inverse();
      tf_T_LI_ = tf_T_CL_.inverse() * tf_T_CI_;
      ROS_WARN_STREAM("IMU to LiDAR translation: [" << tf_T_LI_.getOrigin().x() << ", " << tf_T_LI_.getOrigin().y() << ", "
                                                    << tf_T_LI_.getOrigin().z() << "]");
    } else {
      ROS_ERROR("Did not find LiDAR!");
      return;
    }
    // Left GNSS
    const unsigned int gnssLeftBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("GNSS_L").c_str());
    if (gnssLeftBodyId != std::numeric_limits<unsigned int>::max()) {
      tf_T_CabinGnssL_ = getTransformFromID(gnssLeftBodyId);
      ROS_WARN_STREAM(
          "Left GNSS to Cabin translation: [" << tf_T_CabinGnssL_.getOrigin().x() << ", " << tf_T_CabinGnssL_.getOrigin().y() << ", "
                                              << tf_T_CabinGnssL_.getOrigin().z() << "]");
    } else {
      ROS_ERROR("Did not find left GNSS!");
      return;
    }
    // Right GNSS
    const unsigned int gnssRightBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("GNSS_R").c_str());
    if (gnssRightBodyId != std::numeric_limits<unsigned int>::max()) {
      tf_T_CabinGnssR_ = getTransformFromID(gnssRightBodyId);
      ROS_WARN_STREAM(
          "Right GNSS to Cabin translation: [" << tf_T_CabinGnssR_.getOrigin().x() << ", " << tf_T_CabinGnssR_.getOrigin().y() << ", "
                                               << tf_T_CabinGnssR_.getOrigin().z() << "]");
    } else {
      ROS_ERROR("Did not find right GNSS!");
      return;
    }
    // Base to cabin
    Eigen::Vector3d t = excavatorModelPtr_->getPositionBodyToBody(excavator_model::RD::BodyEnum::BASE,
                                                                  excavator_model::RD::BodyEnum::CABIN,
                                                                  excavator_model::RD::CoordinateFrameEnum::BASE);
    Eigen::Matrix3d R;
    R.setIdentity();
    Eigen::Quaterniond q(R);
    tf_T_CB_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    tf_T_CB_.setOrigin(tf::Vector3(t.x(), t.y(), t.z()));
    ROS_WARN("...found all transformations.");
  }

private:
  // Names
  std::string urdfDescription_;
  std::string baseLinkFrame_;
  std::string imuFrame_;
  std::string lidarFrame_;
  std::string cabinFrame_;
  std::string gnssLeftFrame_;
  std::string gnssRightFrame_;

  // Robot Model
  std::unique_ptr<excavator_model::ExcavatorModel> excavatorModelPtr_;

  // Transformations
  tf::Transform tf_T_LC_;
  tf::Transform tf_T_CL_;
  tf::Transform tf_T_LI_;
  tf::Transform tf_T_IC_;
  tf::Transform tf_T_CI_;
  tf::Transform tf_T_CabinGnssL_;
  tf::Transform tf_T_CabinGnssR_;
  tf::Transform tf_T_CB_;

  // Methods
  tf::Transform getTransformFromID(const unsigned int imuBodyId) {
    tf::Transform tf_T;
    Eigen::Vector3d t = excavatorModelPtr_->getRbdlModel()
        .mFixedBodies[imuBodyId - excavatorModelPtr_->getRbdlModel().fixed_body_discriminator]
        ->mParentTransform.r;
    Eigen::Matrix3d R = excavatorModelPtr_->getRbdlModel()
        .mFixedBodies[imuBodyId - excavatorModelPtr_->getRbdlModel().fixed_body_discriminator]
        ->mParentTransform.E;
    Eigen::Quaterniond q(R);
    tf_T.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    tf_T.setOrigin(tf::Vector3(t.x(), t.y(), t.z()));
    return tf_T;
  }
};

}  // namespace fg_filtering

#endif  // MENZI_SIM_STATICTRANSFORMS_H
