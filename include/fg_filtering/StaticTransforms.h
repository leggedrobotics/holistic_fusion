#ifndef MENZI_SIM_STATICTRANSFORMS_H
#define MENZI_SIM_STATICTRANSFORMS_H

#include "excavator_model/ExcavatorModel.hpp"

namespace fg_filtering {

class StaticTransforms {
 public:
  StaticTransforms(ros::NodeHandle& privateNode) {
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
  void setBaseLinkFrame(const std::string& s) { baseLinkFrame_ = s; }

  void setImuFrame(const std::string& s) { imuFrame_ = s; }

  void setLidarFrame(const std::string& s) { lidarFrame_ = s; }

  void setCabinFrame(const std::string& s) { cabinFrame_ = s; }

  void findTransformations() {
    ROS_WARN("Looking up transformations in URDF model:");
    // Cabin IMU
    const unsigned int imuBodyId = excavatorModelPtr_->getRbdlModel().GetBodyId(std::string("IMU_CABIN_link").c_str());
    if (imuBodyId != std::numeric_limits<unsigned int>::max()) {
      ROS_WARN_STREAM("IMU ID: " << imuBodyId);
      ROS_WARN_STREAM("Numeric limits: " << std::numeric_limits<unsigned int>::max());
      tf_T_CI_ = getTransformFromID(imuBodyId);
      tf_T_IC_ = tf_T_CI_.inverse();
      ROS_WARN_STREAM("IMU_CABIN_LINK: " << tf_T_CI_.getOrigin());
    } else {
      ROS_ERROR("Did not find Cabin IMU!");
      return;
    }

    // LiDAR
  }

 private:
  // Names
  std::string urdfDescription_;
  std::string baseLinkFrame_;
  std::string imuFrame_;
  std::string lidarFrame_;
  std::string cabinFrame_;

  // Robot Model
  std::unique_ptr<excavator_model::ExcavatorModel> excavatorModelPtr_;

  // Transformations
  tf::Transform tf_T_LC_;
  tf::Transform tf_T_CL_;
  tf::Transform tf_T_LI_;
  tf::Transform tf_T_IC_;
  tf::Transform tf_T_CI_;

  // Methods
  tf::Transform getTransformFromID(const unsigned int imuBodyId) {
    tf::Transform T;
    Eigen::Vector3d t = excavatorModelPtr_->getRbdlModel()
                            .mFixedBodies[imuBodyId - excavatorModelPtr_->getRbdlModel().fixed_body_discriminator]
                            ->mParentTransform.r;
    ROS_WARN_STREAM("Translation: " << t);
    Eigen::Matrix3d R = excavatorModelPtr_->getRbdlModel()
                            .mFixedBodies[imuBodyId - excavatorModelPtr_->getRbdlModel().fixed_body_discriminator]
                            ->mParentTransform.E;
    Eigen::Quaterniond q(R);
    T.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    T.setOrigin(tf::Vector3(t.x(), t.y(), t.z()));
    return T;
  }
};

}  // namespace fg_filtering

#endif  // MENZI_SIM_STATICTRANSFORMS_H
