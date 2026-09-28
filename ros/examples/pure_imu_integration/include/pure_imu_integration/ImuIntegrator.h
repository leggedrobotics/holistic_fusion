/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef ImuIntegrator_H
#define ImuIntegrator_H

// std
#include <chrono>

// ROS
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Workspace
#include "holistic_fusion/measurements/UnaryMeasurementXD.h"
#include "holistic_fusion_ros/HolisticFusionRos.h"

// Defined Macros
#define ROS_QUEUE_SIZE 100
#define NUM_GNSS_CALLBACKS_UNTIL_START 20  // 0

namespace imu_integrator {

class ImuIntegrator : public holistic_fusion::HolisticFusionRos {
 public:
  ImuIntegrator(std::shared_ptr<ros::NodeHandle> privateNodePtr);

 private:
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imuPtr) override;
};
}  // namespace imu_integrator
#endif  // end ImuIntegrator_H
