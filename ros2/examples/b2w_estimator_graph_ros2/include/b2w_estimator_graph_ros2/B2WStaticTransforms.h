/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#pragma once

// Workspace
#include <graph_msf_ros2/extrinsics/StaticTransformsTf.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace b2w_se {

class B2WStaticTransforms : public graph_msf::StaticTransformsTf {
 public:
  explicit B2WStaticTransforms(const rclcpp::Node::SharedPtr& nodePtr);


  // Setters
  void setLioOdometryFrame(const std::string& s) { lidarOdometryFrame_ = s; }
  void setGnssFrame(const std::string& s) { gnssFrame_ = s; }
  void setLidarBetweenFrame(const std::string& s) { lidarBetweenFrame_ = s; }
  void setWheelOdometryBetweenFrame(const std::string& s) { wheelOdometryBetweenFrame_ = s; }
  void setWheelLinearVelocityLeftFrame(const std::string& s) { wheelLinearVelocityLeftFrame_ = s; }
  void setWheelLinearVelocityRightFrame(const std::string& s) { wheelLinearVelocityRightFrame_ = s; }
  void setVioOdometryFrame(const std::string& s) { vioOdometryFrame_ = s; }

  // Getters
  const std::string& getLioOdometryFrame() const { return lidarOdometryFrame_; }
  const std::string& getGnssFrame() const { return gnssFrame_; }
  const std::string& getLidarBetweenFrame() const { return lidarBetweenFrame_; }
  const std::string& getWheelOdometryBetweenFrame() const { return wheelOdometryBetweenFrame_; }
  const std::string& getWheelLinearVelocityLeftFrame() const { return wheelLinearVelocityLeftFrame_; }
  const std::string& getWheelLinearVelocityRightFrame() const { return wheelLinearVelocityRightFrame_; }
  const std::string& getVioOdometryFrame() const { return vioOdometryFrame_; }

  // Set flags
  void setUseLioOdometryFlag(bool flag) { useLioOdometryFlag_ = flag; }
  void setUseVioOdometryFlag(bool flag) { useVioOdometryFlag_ = flag; }
  void setUseGnssFlag(bool flag) { useGnssFlag_ = flag; }
  void setUseLioBetweenOdometryFlag(bool flag) { useLioBetweenOdometryFlag_ = flag; }
  void setUseWheelOdometryBetweenFlag(bool flag) { useWheelOdometryBetweenFlag_ = flag; }
  void setUseWheelLinearVelocitiesFlag(bool flag) { useWheelLinearVelocitiesFlag_ = flag; }

 private:
  bool findTransformations() override;

  // Frames
  std::string lidarOdometryFrame_;
  std::string gnssFrame_;
  std::string lidarBetweenFrame_;
  std::string wheelOdometryBetweenFrame_;
  std::string wheelLinearVelocityLeftFrame_;
  std::string wheelLinearVelocityRightFrame_;
  std::string vioOdometryFrame_;

  // Odometry flags
  bool useLioOdometryFlag_ = false;
  bool useGnssFlag_ = false;
  bool useLioBetweenOdometryFlag_ = false;
  bool useVioOdometryFlag_ = false;
  bool useWheelOdometryBetweenFlag_ = false;
  bool useWheelLinearVelocitiesFlag_ = false;
};

}  // namespace b2w_se
