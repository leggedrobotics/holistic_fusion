/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_IMU_IMU_SIGNAL_LOW_PASS_FILTER_HPP
#define GRAPH_MSF_IMU_IMU_SIGNAL_LOW_PASS_FILTER_HPP

#include <cmath>

namespace graph_msf {

class ImuSignalLowPassFilter {
 public:
  ImuSignalLowPassFilter(const double cutoffFrequencyHz, const double samplingTime) {
      setFilterParameters(cutoffFrequencyHz, samplingTime);
  };
  ~ImuSignalLowPassFilter() = default;

  void setFilterParameters(double cutoffFrequencyHz, double samplingTime) {
    cutoffFrequencyRad_ = 2.0 * M_PI * cutoffFrequencyHz;
    samplingTime_ = samplingTime;
    // Calculate filter coefficients
    filteringFactor_ = 1.0 - std::exp(- samplingTime_ * cutoffFrequencyRad_);
//    b0_ = 1.0 - a;
//    b1_ = 0.0;
//    a1_ = -a;
  }

  void reset() {
//    acceleratioin_x1_ = Eigen::Vector3d::Zero();
//    angularVel_x1_ = Eigen::Vector3d::Zero();
    outputAcceleration_y1_ = Eigen::Vector3d::Zero();
    outputAngularVel_y1_ = Eigen::Vector3d::Zero();
  }

  Eigen::Matrix<double, 6, 1> filter(const Eigen::Vector3d& inputAcceleration_x0, const Eigen::Vector3d& inputAngularVel_x0) {
//    Eigen::Vector3d acceleration_y0 = b0_ * acceleration_x0 + b1_ * acceleratioin_x1_ + a1_ * acceleration_y1_;
//    Eigen::Vector3d angularVel_y0 = b0_ * angularVel_x0 + b1_ * angularVel_x1_ + a1_ * angularVel_y1_;
//    acceleratioin_x1_ = acceleration_x0;
//    angularVel_x1_ = angularVel_x0;
//    acceleration_y1_ = acceleration_y0;
//    angularVel_y1_ = angularVel_y0;
    outputAcceleration_y1_ += (inputAcceleration_x0 - outputAcceleration_y1_) * filteringFactor_;
        outputAngularVel_y1_ += (inputAngularVel_x0 - outputAngularVel_y1_) * filteringFactor_;

    // return stacked vectors as 6D vector
    return (Eigen::Matrix<double, 6, 1>() << outputAcceleration_y1_, outputAngularVel_y1_).finished();
  }

 private:
  double cutoffFrequencyRad_;
  double samplingTime_;
  double filteringFactor_;
//  double a1_;
//  double b0_;
//  double b1_;


//  Eigen::Vector3d acceleratioin_x1_ = Eigen::Vector3d::Zero();
//  Eigen::Vector3d angularVel_x1_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d outputAcceleration_y1_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d outputAngularVel_y1_ = Eigen::Vector3d::Zero();
};

}  // namespace graph_msf

#endif  // IMU_MANAGER_HPP_