#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtsam/slam/PriorFactor.h>

#include "graph_msf/config/StaticTransforms.h"
#include "graph_msf/core/GraphManager.h"
#include "graph_msf/interface/constants.h"
#include "graph_msf/core/optimizer/OptimizerLMBatch.hpp"
#include "graph_msf/interface/GraphMsfClassic.h"
#include "graph_msf/interface/GraphMsfHolistic.h"

namespace {

class TestTransforms final : public graph_msf::StaticTransforms {
 public:
  TestTransforms() {
    setImuFrame("imu");
    setInitializationFrame("imu");
    setBaseLinkFrame("imu");
    setWorldFrame("world");
    setOdomFrame("odom");
  }

  bool findTransformations() override { return true; }
};

class TestEstimator final : public graph_msf::GraphMsfClassic, public graph_msf::GraphMsfHolistic {
 public:
  using graph_msf::GraphMsf::pretendFirstMeasurementReceived;
};

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void rejectsInvalidIsamRelinearizationInterval() {
  for (const bool realtime : {false, true}) {
    for (const int skip : {0, -1}) {
      auto config = std::make_shared<graph_msf::GraphConfig>();
      config->realTimeSmootherUseIsamFlag_ = realtime;
      config->useAdditionalSlowBatchSmootherFlag_ = !realtime;
      config->slowBatchSmootherUseIsamFlag_ = true;
      config->relinearizeSkip_ = skip;
      bool rejected = false;
      try {
        graph_msf::GraphManager manager(config, "imu", "world");
      } catch (const std::invalid_argument& error) {
        rejected = std::string(error.what()).find("relinearizeSkip") != std::string::npos;
      }
      require(rejected, "Invalid ISAM2 relinearizeSkip was accepted");
    }
  }
}

void rejectsInvalidStateAndOptimizationCounts() {
  for (const bool state_count : {false, true}) {
    auto config = std::make_shared<graph_msf::GraphConfig>();
    const std::string key = state_count ? "createStateEveryNthImuMeasurement" : "additionalOptimizationIterations";
    if (state_count) {
      config->createStateEveryNthImuMeasurement_ = 0;
    } else {
      config->additionalOptimizationIterations_ = -1;
    }
    TestEstimator estimator;
    bool rejected = false;
    try {
      estimator.setup(config, std::make_shared<TestTransforms>());
    } catch (const std::invalid_argument& error) {
      rejected = std::string(error.what()).find(key) != std::string::npos;
    }
    require(rejected, "Invalid " + key + " was accepted");
  }
}

struct Covariances {
  gtsam::Matrix66 pose;
  gtsam::Matrix33 velocity;
};

Covariances integratedCovariances(double acc_noise, double gyro_noise) {
  auto config = std::make_shared<graph_msf::GraphConfig>();
  config->useImuSignalLowPassFilter_ = false;
  config->realTimeSmootherUseCholeskyFactorizationFlag_ = false;
  config->accNoiseDensity_ = acc_noise;
  config->gyroNoiseDensity_ = gyro_noise;
  graph_msf::GraphManager manager(config, "imu", "world");
  require(manager.initImuIntegrators(config->gravityMagnitude_), "IMU integrator initialization failed");
  require(manager.initPoseVelocityBiasGraph(1.0, gtsam::Pose3(), gtsam::Pose3()), "Prior graph initialization failed");
  auto buffer = std::make_shared<graph_msf::ImuBuffer>(config);
  const Eigen::Vector3d acceleration(0.0, 0.0, config->gravityMagnitude_);
  buffer->addToImuBuffer(0.99, acceleration, Eigen::Vector3d::Zero());
  buffer->addToImuBuffer(1.0, acceleration, Eigen::Vector3d::Zero());
  graph_msf::SafeIntegratedNavState state;
  std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias> optimized_state;
  for (int index = 1; index <= 10; ++index) {
    const double timestamp = 1.0 + 0.01 * index;
    buffer->addToImuBuffer(timestamp, acceleration, Eigen::Vector3d::Zero());
    manager.addImuFactorAndGetState(state, optimized_state, buffer, timestamp, index == 10);
  }
  manager.updateGraph();
  const auto& optimized = manager.getOptimizedGraphState();
  require(std::abs(optimized.ts() - 1.1) < 1e-12, "IMU factor was not optimized");
  return {optimized.poseCovariance(), optimized.velocityCovariance()};
}

void sensorNoiseChangesActualStateCovariance() {
  const auto baseline = integratedCovariances(0.01, 0.02);
  const auto acc = integratedCovariances(0.1, 0.02);
  const auto gyro = integratedCovariances(0.01, 0.1);
  require(acc.velocity.trace() > baseline.velocity.trace() + 1e-4, "Acceleration noise has no covariance effect");
  require(gyro.pose.topLeftCorner<3, 3>().trace() > baseline.pose.topLeftCorner<3, 3>().trace() + 1e-4,
          "Gyroscope noise has no covariance effect");
}

void stationaryPropagationUsesInitializedGravity(bool estimate_gravity) {
  auto config = std::make_shared<graph_msf::GraphConfig>();
  config->imuRate_ = 10.0;
  config->imuBufferLength_ = 20;
  config->useImuSignalLowPassFilter_ = false;
  config->staticAtStartup_ = true;
  config->gyroBiasPrior_ = Eigen::Vector3d::Ones();
  config->estimateGravityFromImuFlag_ = estimate_gravity;
  config->gravityMagnitude_ = estimate_gravity ? 9.81 : 10.0;
  config->minOptimizationFrequency_ = 1e-9;
  config->maxOptimizationFrequency_ = 1e-9;
  TestEstimator estimator;
  estimator.setup(config, std::make_shared<TestTransforms>());
  std::shared_ptr<graph_msf::SafeIntegratedNavState> state;
  std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias> optimized_state;
  Eigen::Matrix<double, 6, 1> measurements;
  const Eigen::Vector3d gyro_bias(0.01, -0.02, 0.03);
  double timestamp = 1.0;
  const auto add_imu = [&]() {
    timestamp += 0.1;
    estimator.addCoreImuMeasurementAndGetState(Eigen::Vector3d(0.0, 0.0, 10.0), gyro_bias, timestamp,
                                             state, optimized_state, measurements);
  };
  for (int index = 0; index < 20 && !estimator.areRollAndPitchInited(); ++index) {
    add_imu();
  }
  require(estimator.areRollAndPitchInited(), "Stationary IMU alignment failed");
  require(config->gyroBiasPrior_.isApprox(gyro_bias), "Static initialization must use the measured gyro-bias mean");
  require(estimator.initYawAndPositionInWorld(0.0, Eigen::Vector3d::Zero(), "imu", "imu"), "Pose initialization failed");
  estimator.pretendFirstMeasurementReceived();
  add_imu();
  require(estimator.isGraphInited(), "Graph initialization failed");

  add_imu();

  require(state != nullptr, "No propagated state returned");
  require(state->getI_v_W_I().norm() < 1e-8, "Stationary propagation uses a stale gravity vector");
}

void rejectsInMotionInitialization() {
  auto config = std::make_shared<graph_msf::GraphConfig>();
  config->staticAtStartup_ = false;
  config->gyroBiasPrior_ = Eigen::Vector3d::Ones();
  TestEstimator estimator;
  bool rejected = false;

  try {
    estimator.setup(config, std::make_shared<TestTransforms>());
  } catch (const std::logic_error& error) {
    const std::string message = error.what();
    rejected = message.find("static_at_startup") != std::string::npos &&
               message.find("not implemented") != std::string::npos;
  }

  require(rejected, "In-motion initialization must fail explicitly during setup");
  require(config->gyroBiasPrior_.isZero(), "In-motion initialization must select a zero gyro-bias mean");
}

void disabledMarginalWindowHandlesSingleState() {
  auto config = std::make_shared<graph_msf::GraphConfig>();
  config->useAdditionalSlowBatchSmootherFlag_ = true;
  config->slowBatchSmootherUseIsamFlag_ = false;
  config->useWindowForMarginalsComputationFlag_ = false;
  graph_msf::OptimizerLMBatch optimizer(config);
  gtsam::NonlinearFactorGraph factors;
  factors.emplace_shared<gtsam::PriorFactor<double>>(0, 2.0, gtsam::noiseModel::Isotropic::Sigma(1, 0.5));
  gtsam::Values values;
  values.insert(0, 1.0);
  optimizer.update(factors, values, {{0, 0.0}});
  optimizer.optimize(10);

  const auto covariance = optimizer.calculateMarginalCovarianceMatrixAtKey(0);

  require(std::abs(covariance(0, 0) - 0.25) < 1e-8, "Whole-graph marginal covariance is incorrect");
}

}  // namespace

int main() {
  try {
    rejectsInvalidIsamRelinearizationInterval();
    rejectsInvalidStateAndOptimizationCounts();
    rejectsInMotionInitialization();
    sensorNoiseChangesActualStateCovariance();
    stationaryPropagationUsesInitializedGravity(false);
    stationaryPropagationUsesInitializedGravity(true);
    disabledMarginalWindowHandlesSingleState();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
