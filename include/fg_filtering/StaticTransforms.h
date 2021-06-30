#ifndef MENZI_SIM_STATICTRANSFORMS_H
#define MENZI_SIM_STATICTRANSFORMS_H

#include "excavator_model/ExcavatorModel.hpp"

namespace fg_filtering {

class StaticTransforms {
public:
  StaticTransforms(ros::NodeHandle &privateNode) {
    std::cout << "Static Transforms container initializing...";
    std::string sParam;
    if (privateNode.getParam("description_name", sParam)) {
      ROS_INFO_STREAM("FactorGraphFiltering - URDF-Description: " << sParam);
      urdfDescription_ = sParam;
    } else {
      ROS_ERROR("FactorGraphFiltering - urdf description not set.");
      return;
    }
    /* load excavator model from URDF */
    double timeStep;
    excavator_model::ExcavatorModel excavatorModel(timeStep);
    excavatorModel.initModelFromUrdf(urdfDescription_.c_str());
    rbdlModel_ = excavatorModel.getRbdlModel();
  }

  // Setters
  void setBaseLinkFrame(const std::string &s) { baseLinkFrame_ = s; }

  void setImuFrame(const std::string &s) { imuFrame_ = s; }

  void setLidarFrame(const std::string &s) { lidarFrame_ = s; }

  void setCabinFrame(const std::string &s) { cabinFrame_ = s; }

  void findTransformations() {
    // Probably better this one
    baseToCabinInBaseFramePosition_ = excavatorModel.getPositionBodyToBody(excavator_model::RD::BodyEnum::BASE,
                                                                           excavator_model::RD::BodyEnum::CABIN,
                                                                           excavator_model::RD::CoordinateFrameEnum::BASE);




    const unsigned int imuBodyId = rbdlModel_.GetBodyId(std::string("IMU_link").c_str());
    if (imuBodyId != std::numeric_limits<unsigned int>::max()) {
      Eigen::Matrix<double, 3, 1> chassisToChassisImuPosition = rbdlModel_.mFixedBodies[imuBodyId - rbdlModel_.fixed_body_discriminator]->mParentTransform.r;
      Eigen::Matrix<double, 3, 3> chassisToChassisImuRotationMatrix = rbdlModel_.mFixedBodies[imuBodyId - rbdlModel_.fixed_body_discriminator]->mParentTransform.E;
    } else {
      ROS_ERROR("Did not find Body IMU!");
      return false;
    }
  }

private:
  // Names
  std::string urdfDescription_;
  std::string baseLinkFrame_;
  std::string imuFrame_;
  std::string lidarFrame_;
  std::string cabinFrame_;

  // Robot Model
  RigidBodyDynamics::Model rbdlModel_;

  // Transformations
  tf::StampedTransform tf_T_LC;
  tf::StampedTransform tf_T_CL;
  tf::StampedTransform tf_T_LI;
  tf::StampedTransform tf_T_IC;
  tf::StampedTransform tf_T_CI;
};

} // namespace fg_filtering

#endif //MENZI_SIM_STATICTRANSFORMS_H
