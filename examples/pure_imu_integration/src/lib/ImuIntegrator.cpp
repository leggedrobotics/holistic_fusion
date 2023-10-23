/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "pure_imu_integration/ImuIntegrator.h"

namespace imu_integrator {

ImuIntegrator::ImuIntegrator(std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  std::cout << YELLOW_START << "ImuIntegrator" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  std::cout << YELLOW_START << "ImuIntegrator" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

// Overridden imuCallback
void ImuIntegrator::imuCallback_(const sensor_msgs::Imu::ConstPtr& imuMsgPtr) {
  sensor_msgs::Imu::Ptr imuMsgPtrCopy = boost::make_shared<sensor_msgs::Imu>(*imuMsgPtr);
  imuMsgPtrCopy->angular_velocity.x = imuMsgPtr->angular_velocity.x * M_PI / 180.0;
  imuMsgPtrCopy->angular_velocity.y = imuMsgPtr->angular_velocity.y * M_PI / 180.0;
  imuMsgPtrCopy->angular_velocity.z = imuMsgPtr->angular_velocity.z * M_PI / 180.0;

  graph_msf::GraphMsfRos::imuCallback_(imuMsgPtrCopy);

  if (graph_msf::GraphMsf::areRollAndPitchInited() && !graph_msf::GraphMsf::areYawAndPositionInited()) {
    std::cout << YELLOW_START << "ImuIntegrator" << GREEN_START
              << " Imu attitude is now initialized: " << graph_msf::GraphMsf::areRollAndPitchInited() << COLOR_END << std::endl;
    graph_msf::GraphMsf::pretendFirstMeasurementReceived();
    graph_msf::GraphMsf::initYawAndPosition(0.0, Eigen::Vector3d::Zero(), staticTransformsPtr_->getWorldFrame(),
                                            staticTransformsPtr_->getImuFrame(), staticTransformsPtr_->getImuFrame());
    std::cout << YELLOW_START << "ImuIntegrator" << GREEN_START << " Set initial yaw and position to zero." << COLOR_END << std::endl;
  }
}

}  // namespace imu_integrator