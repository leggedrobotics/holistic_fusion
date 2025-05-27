/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <boost/optional.hpp>
#include <iostream>
#include <random>

// Package
#include "graph_msf/trajectory_alignment/TrajectoryAlignment.h"

namespace graph_msf {

// Public -------------------------------------------------------------------
TrajectoryAlignment::TrajectoryAlignment() {
  std::cout << YELLOW_START << "TrajectoryAlignment" << GREEN_START << " Created Trajectory Alignment instance." << COLOR_END << std::endl;
}

std::vector<std::pair<double, Eigen::Vector3d>> TrajectoryAlignment::getSe3Trajectory() {
  std::vector<std::pair<double, Eigen::Vector3d>> trajectory;
  std::pair<double, Eigen::Vector3d> onePose;
  for (auto pose : se3Trajectory_.poses()) {
    onePose.first = pose.time();
    onePose.second = pose.position();
    trajectory.push_back(onePose);
  }
  return trajectory;
}

std::vector<std::pair<double, Eigen::Vector3d>> TrajectoryAlignment::getR3Trajectory() {
  std::vector<std::pair<double, Eigen::Vector3d>> trajectory;
  std::pair<double, Eigen::Vector3d> onePose;
  for (auto pose : r3Trajectory_.poses()) {
    onePose.first = pose.time();
    onePose.second = pose.position();
    trajectory.push_back(onePose);
  }
  return trajectory;
}

void TrajectoryAlignment::addSe3Position(Eigen::Vector3d position, double time) {
  const std::lock_guard<std::mutex> alignmentLock(alignmentMutex);
  se3Trajectory_.addPose(position, time);
}

void TrajectoryAlignment::addR3Position(Eigen::Vector3d position, double time) {
  const std::lock_guard<std::mutex> alignmentLock(alignmentMutex);
  r3Trajectory_.addPose(position, time);
}

void TrajectoryAlignment::addR3PositionWithStdDev(Eigen::Vector3d position, double time, Eigen::Vector3d stdDev) {
  const std::lock_guard<std::mutex> alignmentLock(alignmentMutex);
  r3Trajectory_.addPoseWithFilter(position, time, stdDev);
}

bool TrajectoryAlignment::associateTrajectories(
        const Trajectory& trajectoryA,
        const Trajectory& trajectoryB,
        Trajectory&       newTrajectoryA,
        Trajectory&       newTrajectoryB)
{
  newTrajectoryA.clear();
  newTrajectoryB.clear();

  const auto& posesA = trajectoryA.poses();
  const auto& posesB = trajectoryB.poses();
  if (posesA.empty() || posesB.empty()) return false;

  std::size_t i = 0;
  std::size_t j = 0;

  while (i < posesA.size() && j < posesB.size())
  {
    const double tA = posesA[i].time();
    const double tB = posesB[j].time();
    const double dt = tA - tB;

    if (std::fabs(dt) <= 0.15)
    {
      // one-to-one correspondence found
      newTrajectoryA.addPose(posesA[i]);
      newTrajectoryB.addPose(posesB[j]);
      ++i; ++j;
    }
    else
    {
      // advance the trajectory that is behind in time
      (dt < 0.0) ? ++i : ++j;
    }
  }

  return newTrajectoryA.poses().size() >= 3;
}


bool TrajectoryAlignment::trajectoryAlignment(Trajectory& trajectoryA, Trajectory& trajectoryB, Eigen::Isometry3d& returnTransform) {
  // Fill matrices to use Eigen Umeyama function
  const int numberOfMeasurements = std::min(trajectoryA.poses().size(), trajectoryB.poses().size());
  if (numberOfMeasurements < 2) return false;

  Eigen::MatrixXd posesA;
  Eigen::MatrixXd posesB;
  posesA.resize(3, numberOfMeasurements);
  posesB.resize(3, numberOfMeasurements);

  for (unsigned i = 0; i < numberOfMeasurements; ++i) {
    posesA.col(i) = trajectoryA.poses().at(i).position();
    posesB.col(i) = trajectoryB.poses().at(i).position();
  }

  // Umeyama Alignment
  Eigen::Matrix4d transform = umeyama(posesB, posesA, false);
  returnTransform = Eigen::Isometry3d(transform);
  std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Umeyama transform: " << std::endl
            << COLOR_END << returnTransform.matrix() << std::endl;

  return true;
}

bool TrajectoryAlignment::trajectoryAlignmentRobust(
        const Trajectory&  trajectoryA,
        const Trajectory&  trajectoryB,
        Eigen::Isometry3d& returnTransform,
        double             inlierThreshold,   // [m]
        double             ransacConfidence,
        std::size_t        maxIterations,
        bool               withScaling)
{
  // 1. quick sanity checks
  const std::size_t N = std::min(trajectoryA.poses().size(),
                                 trajectoryB.poses().size());
  if (N < 3)
    return false;

  Eigen::Matrix3Xd A(3, N), B(3, N);
  for (std::size_t i = 0; i < N; ++i) {
    A.col(i) = trajectoryA.poses()[i].position();
    B.col(i) = trajectoryB.poses()[i].position();
  }

  // Helper lambdas
  auto umeyama3 = [&](const Eigen::Matrix3Xd& P,
                      const Eigen::Matrix3Xd& Q) -> Eigen::Isometry3d {
    return Eigen::Isometry3d(Eigen::umeyama(P, Q, withScaling));
  };

  auto nonCollinear = [](const Eigen::Vector3d& a,
                         const Eigen::Vector3d& b,
                         const Eigen::Vector3d& c) -> bool {
    return ((b - a).cross(c - a)).squaredNorm() > 1e-9;
  };

  std::mt19937 rng{std::random_device{}()};
  std::uniform_int_distribution<std::size_t> uni(0, N - 1);

  std::vector<std::size_t> bestInliers;
  Eigen::Isometry3d        bestT = Eigen::Isometry3d::Identity();

  const std::size_t sampleSize = 3;
  std::size_t       iter       = 0;
  const double eps = std::numeric_limits<double>::epsilon();

  while (iter < maxIterations) {
    // 1. Random minimal subset (indices)
    std::array<std::size_t, 3> idx;
    do {
      for (auto& k : idx) k = uni(rng);
    } while (!nonCollinear(B.col(idx[0]), B.col(idx[1]), B.col(idx[2])));

    // 2. Extract columns for these indices
    Eigen::Matrix3d B_sample, A_sample;
    for (int j = 0; j < 3; ++j) {
      B_sample.col(j) = B.col(idx[j]);
      A_sample.col(j) = A.col(idx[j]);
    }

    Eigen::Isometry3d T = umeyama3(B_sample, A_sample);

    // 3. Inlier counting
    std::vector<std::size_t> inliers;
    inliers.reserve(N);
    for (std::size_t k = 0; k < N; ++k) {
      double error = (T * B.col(k) - A.col(k)).norm();
      if (error < inlierThreshold) {
        inliers.push_back(k);
      }
    }

    if (inliers.size() > bestInliers.size()) {
      bestInliers.swap(inliers);
      bestT = T;

      double inlierRatio = static_cast<double>(bestInliers.size()) / N;
      double p_no_outliers = 1.0 - std::pow(inlierRatio, sampleSize);
      p_no_outliers = std::clamp(p_no_outliers, eps, 1.0 - eps);
      maxIterations =
          std::min<std::size_t>(maxIterations,
              static_cast<std::size_t>(
                  std::log(1.0 - ransacConfidence) / std::log(p_no_outliers)));
    }
    ++iter;
  }

  if (bestInliers.size() < 3) {
    std::cerr << YELLOW_START << "Trajectory Alignment" << RED_START
              << " Not enough inliers found for robust alignment: "
              << bestInliers.size() << " inliers (need at least 3)."
              << COLOR_END << std::endl;
    return false;
  }

  Eigen::Matrix3Xd Binl(3, bestInliers.size()), Ainl(3, bestInliers.size());
  for (std::size_t c = 0; c < bestInliers.size(); ++c) {
    Binl.col(c) = B.col(bestInliers[c]);
    Ainl.col(c) = A.col(bestInliers[c]);
  }
  returnTransform = umeyama3(Binl, Ainl);

  // Optional IRLS refinement (Huber)
  const int irlsIters = 5;
  const double huberK = 1.345 * inlierThreshold;
  Eigen::VectorXd w(bestInliers.size());

  for (int it = 0; it < irlsIters; ++it) {
    // weights
    for (std::size_t c = 0; c < bestInliers.size(); ++c) {
      double r = (returnTransform * Binl.col(c) - Ainl.col(c)).norm();
      w(c) = (r < huberK) ? 1.0 : huberK / r;
    }
    double wsum = w.sum() + eps;
    Eigen::Vector3d muA = (Ainl.array().rowwise() * w.transpose().array()).rowwise().sum() / wsum;
    Eigen::Vector3d muB = (Binl.array().rowwise() * w.transpose().array()).rowwise().sum() / wsum;

    Eigen::Matrix3d Sigma = Eigen::Matrix3d::Zero();
    for (std::size_t c = 0; c < bestInliers.size(); ++c) {
      Sigma += w(c) * (Binl.col(c) - muB) * (Ainl.col(c) - muA).transpose();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
    if (R.determinant() < 0) {
      Eigen::Matrix3d V = svd.matrixV();
      V.col(2) *= -1;
      R = V * svd.matrixU().transpose();
    }
    double s = 1.0;
    if (withScaling) {
      double num = 0.0, den = 0.0;
      for (std::size_t c = 0; c < bestInliers.size(); ++c) {
        Eigen::Vector3d a = Ainl.col(c) - muA;
        Eigen::Vector3d b = Binl.col(c) - muB;
        num += w(c) * a.dot(R * b);
        den += w(c) * b.squaredNorm();
      }
      s = num / (den + eps);
    }
    Eigen::Vector3d t = muA - s * R * muB;

    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M.block<3,3>(0,0) = s * R;
    M.block<3,1>(0,3) = t;
    returnTransform = Eigen::Isometry3d(M);
  }
  return true;
}

bool TrajectoryAlignment::hasMinimumSpatialSpread(const std::vector<Pose>& poses, double kMinSpread) {
  // Ensure there are enough poses to compute spatial spread
  if (poses.size() < 2) {
    return false;
  }

  // Compute mean of x-y positions
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto& pose : poses) {
    mean += pose.position().head<2>();
  }
  mean /= static_cast<double>(poses.size());

  // Compute covariance matrix of x-y positions
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto& pose : poses) {
    Eigen::Vector2d diff = pose.position().head<2>() - mean;
    covariance += diff * diff.transpose();
  }
  covariance /= static_cast<double>(poses.size());

  // Check smallest eigenvalue (spread) to avoid near-collinear trajectories
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig_solver(covariance);
  return eig_solver.eigenvalues()(0) >= kMinSpread;
}

bool TrajectoryAlignment::alignTrajectories(double& yaw, Eigen::Isometry3d& returnTransform) {
  // Containers for running data processing
  Trajectory copySe3Trajectory;
  Trajectory copyR3Trajectory;
  {
    // Mutex for alignment Flag
    const std::lock_guard<std::mutex> alignmentLock(alignmentMutex);
    // Copy Trajectories
    copySe3Trajectory = se3Trajectory_;
    copyR3Trajectory = r3Trajectory_;
  }

  // Early exit if not enough data
  const size_t minPoses = 10;
  if (copySe3Trajectory.poses().size() < minPoses || copyR3Trajectory.poses().size() < minPoses) {
    std::cerr << YELLOW_START << "Trajectory Alignment" << RED_START
              << " Insufficient data for alignment. SE3 poses: " << copySe3Trajectory.poses().size()
              << ", R3 poses: " << copyR3Trajectory.poses().size()
              << ". At least " << minPoses << " poses required in each trajectory."
              << COLOR_END << std::endl;
    return false;
  }

  // Check for standing still or first alignment
  if (copySe3Trajectory.isStanding(se3Rate_, noMovementTime_, noMovementDistance_) ||
      copyR3Trajectory.isStanding(r3Rate_, noMovementTime_, noMovementDistance_) ||
      firstAlignmentTryFlag_) 
  {
    {
      const std::lock_guard<std::mutex> lock(alignmentMutex);
      se3Trajectory_.clear();
      r3Trajectory_.clear();
      if (firstAlignmentTryFlag_) {
        std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " First Alignment Try." << COLOR_END << std::endl;
        firstAlignmentTryFlag_ = false;
      }
    }
    std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " No movement detected. Trajectories cleared." << COLOR_END << std::endl;
    return false;
  }


  // Status
  if (copyR3Trajectory.poses().size() % 5 == 0) {
    std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Current Distance of Se3 / R3 [m]: " << COLOR_END
              << copySe3Trajectory.distance() << " / " << copyR3Trajectory.distance() << " of required [m] " << minDistanceHeadingInit_
              << std::endl;
  }

  // Perform Checks
  if (copySe3Trajectory.distance() < minDistanceHeadingInit_) {
    return false;
  }
  if (copyR3Trajectory.distance() < minDistanceHeadingInit_) {
    return false;
  }

  double kMinSpread = 0.05;  // Minimum spatial spread in meters
  if (!hasMinimumSpatialSpread(copyR3Trajectory.poses(), kMinSpread)) {
    std::cerr << YELLOW_START << "Trajectory Alignment" << RED_START
              << " Minimum spatial spread check failed for R3 trajectory." << COLOR_END << std::endl;
    return false;
  }

  if (!hasMinimumSpatialSpread(copySe3Trajectory.poses(), kMinSpread)) {
    std::cerr << YELLOW_START << "Trajectory Alignment" << RED_START
              << " Minimum spatial spread check failed for SE3 trajectory." << COLOR_END << std::endl;
    return false;
  }

  // Status that all checks passed
  std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " All checks passed. Trying to align." << COLOR_END << std::endl;

  // Cutting of Timestamps Outside the Joint Time Window
  double startTime = std::max(copySe3Trajectory.poses().front().time(), copyR3Trajectory.poses().front().time());
  double endTime = std::min(copySe3Trajectory.poses().back().time(), copyR3Trajectory.poses().back().time());
  // Print
  std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Trajectories A and B before cutting of size: " << COLOR_END
            << copySe3Trajectory.poses().size() << " / " << copyR3Trajectory.poses().size() << std::endl;
  // Cut
  copySe3Trajectory.cutTrajectory(startTime, endTime);
  copyR3Trajectory.cutTrajectory(startTime, endTime);
  // Print
  std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START
            << " Trajectories A and B after cutting of size and time: " << COLOR_END << copySe3Trajectory.poses().size() << " / "
            << copyR3Trajectory.poses().size() << std::endl;
  // Downsample the longer trajectory to the shorter one
  int downsampleFactor = 1;
  if (copySe3Trajectory.poses().size() > copyR3Trajectory.poses().size()) {
    downsampleFactor = std::floor(copySe3Trajectory.poses().size() / copyR3Trajectory.poses().size());
    copySe3Trajectory.downsample(downsampleFactor);
  } else {
    downsampleFactor = std::floor(copyR3Trajectory.poses().size() / copySe3Trajectory.poses().size());
    copyR3Trajectory.downsample(downsampleFactor);
  }
  std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Downsample Factor: " << COLOR_END << downsampleFactor
            << std::endl;
  // Print
  std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Trajectories A and B after downsampling: " << COLOR_END
            << copySe3Trajectory.poses().size() << " / " << copyR3Trajectory.poses().size() << std::endl;

  // Align Trajectories
  Trajectory newSe3Trajectory;
  Trajectory newR3Trajectory;
  // Associate Trajectories
  // Difference of trajectory size should not be larger than 10%
  bool sizeDeviatesTooMuch = (std::abs(static_cast<int>(copySe3Trajectory.poses().size() - copyR3Trajectory.poses().size())) >
                              0.1 * std::min(copySe3Trajectory.poses().size(), copyR3Trajectory.poses().size()));
  // If the trajectories deviate too much, associate them
  if (sizeDeviatesTooMuch) {
    bool success = associateTrajectories(copySe3Trajectory, copyR3Trajectory, newSe3Trajectory, newR3Trajectory);
    if (!success) {
      std::cerr << YELLOW_START << "Trajectory Alignment" << RED_START << " Trajectory Association failed." << COLOR_END << std::endl;
      return false;
    }
    std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START
              << " Trajectories Associated, as after downsampling they were different in size." << COLOR_END << std::endl;
  }
  // If the trajectories do not deviate too much, just copy them
  else {
    newSe3Trajectory = copySe3Trajectory;
    newR3Trajectory = copyR3Trajectory;
    std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START
              << " Trajectories copied, as after downsampling they were similar in size." << COLOR_END << std::endl;
  }

  // Update Trajectories
  copySe3Trajectory = newSe3Trajectory;
  copyR3Trajectory = newR3Trajectory;
  Eigen::Isometry3d alignmentTransform;
  // // Trajectory Alignment
  // if (!trajectoryAlignment(newR3Trajectory, newSe3Trajectory, alignmentTransform)) {
  //   std::cout << "TrajectoryAlignment::initializeYaw trajectoryAlignment failed." << std::endl;
  //   return false;
  // } else {
  //   std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Trajectories Aligned." << COLOR_END << std::endl;
  // }

  Eigen::Isometry3d att;
  bool ok = trajectoryAlignmentRobust(newR3Trajectory, newSe3Trajectory, alignmentTransform,
                                              /*inlierThreshold*/0.15,
                                              /*confidence*/0.999,
                                              /*maxIter*/1000,
                                              /*withScaling*/false);
  if (!ok) {
    std::cout << "TrajectoryAlignment::initializeYaw trajectoryAlignment failed." << std::endl;
    return false;
  } else {
    std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Trajectories Aligned." << COLOR_END << std::endl;
  }

  // Math --> Rotation to Yaw
  Eigen::Matrix3d rotation = alignmentTransform.rotation();
  double pitch = -asin(rotation(2, 0));
  double roll = atan2(rotation(2, 1), rotation(2, 2));
  yaw = atan2(rotation(1, 0) / cos(pitch), rotation(0, 0) / cos(pitch));
  std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Initial Roll/Pitch/Yaw(deg):" << COLOR_END << roll * 180 / M_PI
            << "," << pitch * 180 / M_PI << "," << yaw * 180 / M_PI << std::endl;
  // Return
  returnTransform = alignmentTransform;
  return true;
}

}  // namespace graph_msf
