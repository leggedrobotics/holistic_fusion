/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Gtsam
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/expressions.h>

int main(int argc, char** argv) {
  // Gtsam nonlinear graph
  // gtsam::NonlinearFactorGraph graph;
  gtsam::ExpressionFactorGraph graph;

  // Values
  gtsam::Values initialValues;

  // Add prior on the first key
  auto priorModel = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
  gtsam::Key graphKey = 0;

  // Measurements
  // GNSS in world frame
  gtsam::Pose3 T_W_I_0 = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
  gtsam::Pose3 T_W_I_1 = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(1.0, 0.0, 0.0));
  gtsam::Pose3 T_W_I_2 = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(2.0, 0.0, 0.0));
  gtsam::Pose3 T_M_W(gtsam::Rot3::RzRyRx(0.0, 0.0, M_PI / 2.0), gtsam::Point3(0.0, 0.0, 0.0));
  // LiDAR in map frame
  gtsam::Pose3 T_M_I_0 = T_M_W * T_W_I_0;
  gtsam::Pose3 T_M_I_1 = T_M_W * T_W_I_1;
  gtsam::Pose3 T_M_I_2 = T_M_W * T_W_I_2;

  // Expressions
  gtsam::Pose3_ T_M_W_(gtsam::symbol_shorthand::T(graphKey));
  gtsam::Pose3_ x0_(gtsam::symbol_shorthand::X(0));
  gtsam::Pose3_ x1_(gtsam::symbol_shorthand::X(graphKey + 1));
  gtsam::Pose3_ x2_(gtsam::symbol_shorthand::X(graphKey + 2));

  // Add measurements and values
  graph.addPrior(gtsam::symbol_shorthand::X(graphKey), T_W_I_0, priorModel);
  graph.addExpressionFactor(gtsam::Pose3_(T_M_W_ * x0_), T_M_I_0, priorModel);
  // graph.addPrior(gtsam::symbol_shorthand::X(graphKey), lidarPose0, priorModel);
  initialValues.insert(gtsam::symbol_shorthand::X(graphKey), T_W_I_0);
  initialValues.insert(gtsam::symbol_shorthand::T(graphKey), gtsam::Pose3());
  ++graphKey;
  graph.addPrior(gtsam::symbol_shorthand::X(graphKey), T_W_I_1, priorModel);
  graph.addExpressionFactor(gtsam::Pose3_(T_M_W_ * x1_), T_M_I_1, priorModel);
  // graph.addPrior(gtsam::symbol_shorthand::X(graphKey), lidarPose1, priorModel);
  initialValues.insert(gtsam::symbol_shorthand::X(graphKey), T_W_I_0);
  ++graphKey;
  gtsam::PriorFactor<gtsam::Pose3> priorFactor(gtsam::symbol_shorthand::X(graphKey), T_W_I_2, priorModel);
  graph.add(priorFactor);
  gtsam::ExpressionFactor<gtsam::Pose3> expressionFactor(priorModel, T_M_I_2, gtsam::Pose3_(T_M_W_ * x2_));
  graph.add(expressionFactor);
  // graph.addPrior(gtsam::symbol_shorthand::X(graphKey), lidarPose2, priorModel);
  initialValues.insert(gtsam::symbol_shorthand::X(graphKey), T_W_I_0);

  // Optimize
  gtsam::LevenbergMarquardtParams params;
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialValues, params);
  gtsam::Values result = optimizer.optimize();
  std::cout << "Optimization complete." << std::endl;

  // Conclude
  std::cout << "initial error=" << graph.error(initialValues) << std::endl;
  std::cout << "final error=" << graph.error(result) << std::endl;

  // Print result
  std::cout << "----------------------------------------\n";
  result.print("Final result:\n");

  // Print ground truth vs optimized map to world
  std::cout << "----------------------------------------\n";
  std::cout << "Ground truth map to world:\n";
  std::cout << T_M_W.matrix() << std::endl;
  std::cout << "Optimized map to world:\n";
  std::cout << result.at<gtsam::Pose3>(gtsam::symbol_shorthand::T(0)).matrix() << std::endl;

  return 0;
}
