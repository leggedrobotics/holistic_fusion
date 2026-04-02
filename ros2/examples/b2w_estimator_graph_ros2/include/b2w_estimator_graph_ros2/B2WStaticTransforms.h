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
  void setLioOdometryFrame(const std::string& s) { 
    lidarOdometryFrame_ = s;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("B2WStaticTransforms"), "\033[92mSetting lidarOdometryFrame_ = " << s << "\033[0m");
  }
  void setGnssFrame(const std::string& s) { 
    gnssFrame_ = s;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("B2WStaticTransforms"), "\033[92mSetting gnssFrame_ = " << s << "\033[0m");
  }
  void setLidarBetweenFrame(const std::string& s) { 
    lidarBetweenFrame_ = s;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("B2WStaticTransforms"), "\033[92mSetting lidarBetweenFrame_ = " << s << "\033[0m");
  }
  void setVioOdometryFrame(const std::string& s) { 
    vioOdometryFrame_ = s;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("B2WStaticTransforms"), "\033[92mSetting vioOdometryFrame_ = " << s << "\033[0m");
  }

  void setVioOdometryBetweenFrame(const std::string& s) { 
    vioOdometryBetweenFrame_ = s;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("B2WStaticTransforms"), "\033[92mSetting vioOdometryBetweenFrame_ = " << s << "\033[0m");
  }

  // Getters
  const std::string& getLioOdometryFrame() const { return lidarOdometryFrame_; }
  const std::string& getGnssFrame() const { return gnssFrame_; }
  const std::string& getLidarBetweenFrame() const { return lidarBetweenFrame_; }
  const std::string& getVioOdometryFrame() const { return vioOdometryFrame_; }
  const std::string& getVioOdometryBetweenFrame() const { return vioOdometryBetweenFrame_; }


  // Set flags
  void setUseLioOdometryFlag(bool flag) { useLioOdometryFlag_ = flag; }
  void setUseVioOdometryFlag(bool flag) { useVioOdometryFlag_ = flag; }
  void setUseVioOdometryBetweenFlag(bool flag) { useVioOdometryBetweenFlag_ = flag; }
  void setUseGnssFlag(bool flag) { useGnssFlag_ = flag; }
  void setUseLioBetweenOdometryFlag(bool flag) { useLioBetweenOdometryFlag_ = flag; }

 private:
  bool findTransformations() override;

  // Frames
  std::string lidarOdometryFrame_ = "";
  std::string gnssFrame_ = "";
  std::string lidarBetweenFrame_ = "";
  std::string vioOdometryFrame_ = "";
  std::string vioOdometryBetweenFrame_ = "";

  // Odometry flags
  bool useLioOdometryFlag_ = false;
  bool useGnssFlag_ = false;
  bool useLioBetweenOdometryFlag_ = false;
  bool useVioOdometryFlag_ = false;
  bool useVioOdometryBetweenFlag_ = false;
};

}  // namespace b2w_se
