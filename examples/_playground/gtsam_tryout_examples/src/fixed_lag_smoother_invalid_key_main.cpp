/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Gtsam
// 1. Generic Datatypes
// 1.1 Symbol Shorthand
#include <gtsam/inference/Symbol.h>
// 1.2 Pose3
#include <gtsam/geometry/Pose3.h>
// 2. Optimizers
// 2.1 Standard Batch
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include "gtsam/nonlinear/ISAM2.h"
// 2.2 Incremental Fixed Lag Smoother
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
// 3. Factors
// 3.1 IMU
#include <gtsam/navigation/CombinedImuFactor.h>
// 3.2 Between Factor
#include <gtsam/slam/BetweenFactor.h>
// 3.3 Prior Factor
#include <gtsam/slam/PriorFactor.h>

int main(int argc, char** argv) {
  // Gtsam nonlinear graph
  gtsam::NonlinearFactorGraph graph;

  // Values
  gtsam::Values initialValues;

  // Noise Models
  auto priorModel = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
  auto betweenModel = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

  // Measurements
  // GNSS in world frame
  gtsam::Pose3 T_W_I0 = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
  gtsam::Pose3 T_I0_I1 = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(1.0, 0.0, 0.0));
  gtsam::Pose3 T_I1_I2 = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(1.0, 0.0, 0.0));
  gtsam::Pose3 T_I2_I3 = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(1.0, 0.0, 0.0));
  gtsam::Pose3 T_I3_I4 = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(1.0, 0.0, 0.0));

  // Add measurements and values
  // Initial Pose
  gtsam::Key graphKey = 0;
  graph.addPrior(gtsam::symbol_shorthand::X(graphKey), T_W_I0, priorModel);
  initialValues.insert(gtsam::symbol_shorthand::X(graphKey), T_W_I0);

  // Between Poses
  // 0 -> 1
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(graphKey), gtsam::symbol_shorthand::X(graphKey + 1), T_I0_I1,
                                               betweenModel));
  // Delta noise in tangent space
  gtsam::Pose3 deltaNoise = gtsam::Pose3::Expmap((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished());
  initialValues.insert(gtsam::symbol_shorthand::X(graphKey + 1), T_W_I0.compose(T_I0_I1).compose(deltaNoise));
  // 1 -> 2
  ++graphKey;
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(graphKey), gtsam::symbol_shorthand::X(graphKey + 1), T_I1_I2,
                                               betweenModel));
  initialValues.insert(gtsam::symbol_shorthand::X(graphKey + 1), T_W_I0.compose(T_I0_I1.compose(T_I1_I2)).compose(deltaNoise));
  // 2 -> 3
  ++graphKey;
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(graphKey), gtsam::symbol_shorthand::X(graphKey + 1), T_I2_I3,
                                               betweenModel));
  initialValues.insert(gtsam::symbol_shorthand::X(graphKey + 1),
                       T_W_I0.compose(T_I0_I1.compose(T_I1_I2.compose(T_I2_I3))).compose(deltaNoise));
  // 3 -> 4
  ++graphKey;
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(graphKey), gtsam::symbol_shorthand::X(graphKey + 1), T_I3_I4,
                                               betweenModel));
  initialValues.insert(gtsam::symbol_shorthand::X(graphKey + 1),
                       T_W_I0.compose(T_I0_I1.compose(T_I1_I2.compose(T_I2_I3.compose(T_I3_I4)))).compose(deltaNoise));

  // Optimize --> Classic
  gtsam::LevenbergMarquardtParams params;
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialValues, params);
  gtsam::Values lmResult = optimizer.optimize();
  std::cout << "Classic LM-Optimization complete." << std::endl;

  // Conclude
  std::cout << "initial error=" << graph.error(initialValues) << std::endl;
  std::cout << "final error=" << graph.error(lmResult) << std::endl;

  // Same with ISAM2
  gtsam::ISAM2Params isam2Params;
  gtsam::ISAM2 isam2(isam2Params);
  auto isam2Result = isam2.update(graph, initialValues);
  std::cout << "ISAM2 Optimization complete." << std::endl;

  // Conclude
  std::cout << "initial error=" << graph.error(initialValues) << std::endl;

  // Now with IncrementalFixedLagSmoother
  // Case 1: All in smoother lag
  gtsam::IncrementalFixedLagSmoother fixedLagSmoother1(3, isam2Params);
  std::map<gtsam::Key, double> allInGraphKeysTimeStampMap;
  // 0->2s
  for (size_t i = 0; i < 3; ++i) {
    allInGraphKeysTimeStampMap.insert(std::make_pair(gtsam::symbol_shorthand::X(i), i));
  }
  auto fixedLagResult = fixedLagSmoother1.update(graph, initialValues, allInGraphKeysTimeStampMap);
  // 2->4s
  for (size_t i = 3; i < 5; ++i) {
    allInGraphKeysTimeStampMap.insert(std::make_pair(gtsam::symbol_shorthand::X(i), i));
  }
  std::cout << "Step by step IncrementalFixedLagSmoother Optimization complete." << std::endl;
  // Conclude
  std::cout << "initial error=" << graph.error(initialValues) << std::endl;

  // Case 2: Not in smoother lag
  gtsam::IncrementalFixedLagSmoother fixedLagSmoother2(3, isam2Params);
  std::map<gtsam::Key, double> notInGraphKeysTimeStampMap;
  // 0->4s
  for (size_t i = 0; i < 5; ++i) {
    notInGraphKeysTimeStampMap.insert(std::make_pair(gtsam::symbol_shorthand::X(i), i));
  }
  auto fixedLagResult2 = fixedLagSmoother2.update(graph, initialValues, notInGraphKeysTimeStampMap);
  std::cout << "All at once IncrementalFixedLagSmoother Optimization complete." << std::endl;
  // Conclude
  std::cout << "initial error=" << graph.error(initialValues) << std::endl;

  // Return
  return 0;
}
