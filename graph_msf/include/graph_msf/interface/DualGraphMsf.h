/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_H
#define GRAPH_MSF_H

// Package
#include "graph_msf/interface/GraphMsf.h"

// Defined macros
#define ROS_QUEUE_SIZE 100
#define REQUIRED_GNSS_NUM_NOT_JUMPED 20          // 2*singleGnssJumping = 2*20 = 40
#define GNSS_COVARIANCE_VIOLATION_THRESHOLD 0.2  // 10000
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define RED_START "\033[31m"
#define COLOR_END "\033[0m"

namespace graph_msf {

// Actual Class
class DualGraphMsf : GraphMsf {
 public:  // Interface
  // Constructor
  DualGraphMsf();
  // Destructor
  ~DualGraphMsf() = default;
  // Setup
  virtual bool setup();

  // Graph Manipulation
  void activateFallbackGraph();

  // Adder functions
  /// Return
  std::shared_ptr<SafeNavState> addDualOdometryMeasurementAndReturnNavState(const UnaryMeasurement6D& odometryKm1,
                                                                            const UnaryMeasurement6D& odometryK,
                                                                            const Eigen::Matrix<double, 6, 1>& poseBetweenNoise);
  /// No return
  void addDualGnssPositionMeasurement(const UnaryMeasurement3D& W_t_W_frame, const Eigen::Vector3d& lastPosition,
                                      const Eigen::Vector3d& gnssCovarianceXYZ, const bool attemptGraphSwitching,
                                      const bool addedYawBefore);

 protected:
  // Methods -------------
  /// Worker functions
  //// Updating the factor graph
  void optimizeGraph_();
  //// GNSS Violation
  bool isGnssCovarianceViolated_(const Eigen::Vector3d& gnssCovarianceXYZ);

  /// Utility functions
  //// Geometric transformation to IMU in world frame
  Eigen::Vector3d W_t_W_Frame1_to_W_t_W_Frame2_(const Eigen::Vector3d& W_t_W_frame1, const std::string& frame1, const std::string& frame2,
                                                const Eigen::Matrix3d& R_W_frame2);

 private:  // Variables -------------
  // Factor graph
  std::shared_ptr<GraphManager> graphMgrPtr_ = nullptr;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_H
