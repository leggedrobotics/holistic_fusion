/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_CONVERSIONS_H
#define GRAPH_MSF_CONVERSIONS_H

namespace graph_msf {

/**
 * @brief Convert a 6x6 covariance matrix from ROS convention to GTSAM convention.
 * @param covGtsam The 6x6 covariance matrix in GTSAM convention.
 * @return The 6x6 covariance matrix in ROS convention.
 */
Eigen::Matrix<double, 6, 6> convertCovarianceGtsamConventionToRosConvention(const Eigen::Matrix<double, 6, 6>& covGtsam) {
  Eigen::Matrix<double, 6, 6> covRos;
  covRos.setZero();
  covRos.block<3, 3>(0, 0) = covGtsam.block<3, 3>(3, 3);
  covRos.block<3, 3>(3, 3) = covGtsam.block<3, 3>(0, 0);
  covRos.block<3, 3>(0, 3) = covGtsam.block<3, 3>(3, 0);
  covRos.block<3, 3>(3, 0) = covGtsam.block<3, 3>(0, 3);
  return covRos;
}

}  // namespace graph_msf

#endif  // GRAPH_MSF_CONVERSIONS_H
