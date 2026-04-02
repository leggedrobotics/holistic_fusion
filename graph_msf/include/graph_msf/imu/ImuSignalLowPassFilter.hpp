/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_IMU_IMU_SIGNAL_LOW_PASS_FILTER_HPP
#define GRAPH_MSF_IMU_IMU_SIGNAL_LOW_PASS_FILTER_HPP

#include <Eigen/Dense>
#include <cmath>

namespace graph_msf {

/**
 * @brief Second-order Butterworth low-pass filter for IMU signals
 *
 * This filter provides superior characteristics compared to first-order filters:
 * - Steeper roll-off (40 dB/decade vs 20 dB/decade)
 * - Maximally flat passband response
 * - Clean cutoff frequency with minimal ripple
 * - Better attenuation of high-frequency noise
 */
class ImuSignalLowPassFilter {
 public:
  ImuSignalLowPassFilter(const double cutoffFrequencyHz, const double samplingTime) {
    setFilterParameters(cutoffFrequencyHz, samplingTime);
  };
  ~ImuSignalLowPassFilter() = default;

  void setFilterParameters(double cutoffFrequencyHz, double samplingTime) {
    cutoffFrequencyHz_ = cutoffFrequencyHz;
    samplingTime_ = samplingTime;

    // Calculate second-order Butterworth filter coefficients
    // Using bilinear transform (Tustin's method) for stability
    const double nyquistFreq = 1.0 / (2.0 * samplingTime_);
    const double normalizedCutoff = cutoffFrequencyHz_ / nyquistFreq;

    // Prevent numerical issues for very high cutoff frequencies
    if (normalizedCutoff >= 1.0) {
      // If cutoff is at or above Nyquist frequency, disable filtering
      b0_ = 1.0;
      b1_ = 0.0;
      b2_ = 0.0;
      a1_ = 0.0;
      a2_ = 0.0;
      return;
    }

    // Second-order Butterworth filter design
    const double wc = std::tan(M_PI * normalizedCutoff);
    const double wc2 = wc * wc;
    const double sqrt2 = std::sqrt(2.0);
    const double k1 = sqrt2 * wc;
    const double k2 = wc2;
    const double a0 = k2 + k1 + 1.0;

    // Normalized coefficients
    b0_ = k2 / a0;
    b1_ = 2.0 * k2 / a0;
    b2_ = k2 / a0;
    a1_ = (2.0 * k2 - 2.0) / a0;
    a2_ = (k2 - k1 + 1.0) / a0;
  }

  void reset() {
    // Reset acceleration filter state
    x1_accel_ = Eigen::Vector3d::Zero();
    x2_accel_ = Eigen::Vector3d::Zero();
    y1_accel_ = Eigen::Vector3d::Zero();
    y2_accel_ = Eigen::Vector3d::Zero();

    // Reset angular velocity filter state
    x1_gyro_ = Eigen::Vector3d::Zero();
    x2_gyro_ = Eigen::Vector3d::Zero();
    y1_gyro_ = Eigen::Vector3d::Zero();
    y2_gyro_ = Eigen::Vector3d::Zero();
  }

  Eigen::Matrix<double, 6, 1> filter(const Eigen::Vector3d& inputAcceleration_x0, const Eigen::Vector3d& inputAngularVel_x0) {
    // Apply second-order filter to acceleration
    Eigen::Vector3d outputAcceleration = applyFilter(inputAcceleration_x0, x1_accel_, x2_accel_, y1_accel_, y2_accel_);

    // Apply second-order filter to angular velocity
    Eigen::Vector3d outputAngularVel = applyFilter(inputAngularVel_x0, x1_gyro_, x2_gyro_, y1_gyro_, y2_gyro_);

    // Return stacked vectors as 6D vector
    return (Eigen::Matrix<double, 6, 1>() << outputAcceleration, outputAngularVel).finished();
  }

  /**
   * @brief Get the actual cutoff frequency of the filter
   * @return Cutoff frequency in Hz
   */
  double getCutoffFrequency() const { return cutoffFrequencyHz_; }

  /**
   * @brief Get the sampling time
   * @return Sampling time in seconds
   */
  double getSamplingTime() const { return samplingTime_; }

 private:
  /**
   * @brief Apply second-order filter to a 3D vector
   * @param input Current input sample
   * @param x1 Previous input sample (updated)
   * @param x2 Second previous input sample (updated)
   * @param y1 Previous output sample (updated)
   * @param y2 Second previous output sample (updated)
   * @return Filtered output
   */
  Eigen::Vector3d applyFilter(const Eigen::Vector3d& input, Eigen::Vector3d& x1, Eigen::Vector3d& x2, Eigen::Vector3d& y1,
                              Eigen::Vector3d& y2) {
    // Direct Form II implementation of second-order filter
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    Eigen::Vector3d output = b0_ * input + b1_ * x1 + b2_ * x2 - a1_ * y1 - a2_ * y2;

    // Update state variables
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;

    return output;
  }

  // Filter parameters
  double cutoffFrequencyHz_;
  double samplingTime_;

  // Filter coefficients (second-order Butterworth)
  double b0_, b1_, b2_;  // Numerator coefficients
  double a1_, a2_;       // Denominator coefficients (a0 = 1)

  // Filter state for acceleration (3D)
  Eigen::Vector3d x1_accel_ = Eigen::Vector3d::Zero();  // Previous input
  Eigen::Vector3d x2_accel_ = Eigen::Vector3d::Zero();  // Second previous input
  Eigen::Vector3d y1_accel_ = Eigen::Vector3d::Zero();  // Previous output
  Eigen::Vector3d y2_accel_ = Eigen::Vector3d::Zero();  // Second previous output

  // Filter state for angular velocity (3D)
  Eigen::Vector3d x1_gyro_ = Eigen::Vector3d::Zero();  // Previous input
  Eigen::Vector3d x2_gyro_ = Eigen::Vector3d::Zero();  // Second previous input
  Eigen::Vector3d y1_gyro_ = Eigen::Vector3d::Zero();  // Previous output
  Eigen::Vector3d y2_gyro_ = Eigen::Vector3d::Zero();  // Second previous output
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_IMU_IMU_SIGNAL_LOW_PASS_FILTER_HPP