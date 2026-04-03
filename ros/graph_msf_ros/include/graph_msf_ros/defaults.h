/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_ROS_DEFAULTS_H
#define GRAPH_MSF_ROS_DEFAULTS_H

namespace graph_msf {
inline constexpr double kDefaultTfLookupTimeoutSeconds = 3.0;
inline constexpr double kStaticTransformRetrySleepSeconds = 0.1;
}  // namespace graph_msf

#endif  // GRAPH_MSF_ROS_DEFAULTS_H
