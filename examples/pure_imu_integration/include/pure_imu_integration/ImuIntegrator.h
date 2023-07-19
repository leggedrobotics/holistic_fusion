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
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros/GraphMsfRos.h"

// Defined Macros
#define ROS_QUEUE_SIZE 100
#define NUM_GNSS_CALLBACKS_UNTIL_START 20  // 0

namespace imu_integrator {

class ImuIntegrator : public graph_msf::GraphMsfRos {
 public:
  ImuIntegrator(std::shared_ptr<ros::NodeHandle> privateNodePtr);

 private:
  virtual void imuCallback_(const sensor_msgs::Imu::ConstPtr& imuPtr) override;
};
}  // namespace imu_integrator
#endif  // end ImuIntegrator_H
