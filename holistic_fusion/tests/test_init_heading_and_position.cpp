#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>

#include "holistic_fusion/config/StaticTransforms.h"
#include "holistic_fusion/interface/HolisticFusionClassic.h"
#include "holistic_fusion/interface/HolisticFusionHolistic.h"

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

// Sensor frame S with a strongly tilted mount on the IMU, where a yaw transfer that ignores the tilt fails
const gtsam::Pose3 kT_I_S(gtsam::Rot3::Ypr(0.5, 0.6, M_PI), gtsam::Point3(0.2, -0.1, 0.3));

class TestTransforms final : public holistic_fusion::StaticTransforms {
 public:
  TestTransforms() {
    setImuFrame("imu");
    setInitializationFrame("imu");
    setBaseLinkFrame("imu");
    setWorldFrame("world");
    setOdomFrame("odom");
    set_T_frame1_frame2_andInverse("imu", "sensor", Eigen::Isometry3d(kT_I_S.matrix()));
  }

  bool findTransformations() override { return true; }
};

class TestEstimator final : public holistic_fusion::HolisticFusionClassic, public holistic_fusion::HolisticFusionHolistic {};

// Aligns a static IMU with the true orientation R_W_I, then runs the init and returns the initialized IMU pose in the world
gtsam::Pose3 initializeImuPose(const gtsam::Rot3& R_W_I, const std::function<bool(TestEstimator&)>& init) {
  auto config = std::make_shared<holistic_fusion::GraphConfig>();
  config->imuRate_ = 10.0;
  config->imuBufferLength_ = 20;
  config->useImuSignalLowPassFilter_ = false;
  config->staticAtStartup_ = true;
  config->gyroBiasPrior_ = Eigen::Vector3d::Zero();
  config->estimateGravityFromImuFlag_ = true;
  config->optimizeReferenceFramePosesWrtWorldFlag_ = true;
  TestEstimator estimator;
  estimator.setup(config, std::make_shared<TestTransforms>());

  std::shared_ptr<holistic_fusion::SafeIntegratedNavState> state;
  std::shared_ptr<holistic_fusion::SafeNavStateWithCovarianceAndBias> optimizedState;
  Eigen::Matrix<double, 6, 1> measurements;
  const Eigen::Vector3d I_specificForce = R_W_I.unrotate(gtsam::Point3(0.0, 0.0, 9.81));
  double timestamp = 1.0;
  const auto addImu = [&]() {
    timestamp += 0.1;
    estimator.addCoreImuMeasurementAndGetState(I_specificForce, Eigen::Vector3d::Zero(), timestamp, state, optimizedState, measurements);
  };
  for (int index = 0; index < 20 && !estimator.areRollAndPitchInited(); ++index) {
    addImu();
  }
  require(estimator.areRollAndPitchInited(), "Stationary IMU alignment failed");
  require(init(estimator), "Initialization failed");
  require(!init(estimator), "A second initialization must fail");
  addImu();
  return gtsam::Pose3(state->getT_W_Ik().matrix());
}

holistic_fusion::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> poseMeasurement(const gtsam::Pose3& T_M_S,
                                                                                  const std::string& fixedFrame) {
  return {"pose",
          10,
          "sensor",
          "sensor_corrected",
          holistic_fusion::RobustNorm::None(),
          1.0,
          1.0,
          Eigen::Isometry3d(T_M_S.matrix()),
          Eigen::Matrix<double, 6, 1>::Ones(),
          fixedFrame,
          "world",
          Eigen::Matrix<double, 6, 1>::Ones()};
}

double headingError(const gtsam::Rot3& R_W_S, const gtsam::Rot3& R_W_Smeas) {
  return gtsam::Rot3::Logmap(R_W_S * R_W_Smeas.inverse()).z();
}

// The heading of S must match the measurement in every attitude, also if the measured tilt differs slightly from the IMU
void headingInitMatchesMeasuredPoseOfSensor() {
  const gtsam::Rot3 xAxisUp = gtsam::Rot3::Ry(-M_PI_2);
  const std::vector<gtsam::Rot3> sensorAttitudes{gtsam::Rot3::Ypr(2.1, 0.2, -0.4), gtsam::Rot3::Rz(-2.8) * xAxisUp,
                                                 gtsam::Rot3::Rz(0.4) * gtsam::Rot3::Rx(M_PI)};
  for (const gtsam::Rot3& R_W_S : sensorAttitudes) {
    for (const gtsam::Rot3& tiltError : {gtsam::Rot3(), gtsam::Rot3::Rx(0.02) * gtsam::Rot3::Ry(-0.015)}) {
      const gtsam::Pose3 T_W_Smeas(tiltError * R_W_S, gtsam::Point3(1.0, -2.0, 0.5));
      const gtsam::Pose3 T_W_I = initializeImuPose(R_W_S * kT_I_S.rotation().inverse(), [&](TestEstimator& estimator) {
        return estimator.initHeadingAndPosition(poseMeasurement(T_W_Smeas, "world"));
      });
      const gtsam::Pose3 T_W_S = T_W_I * kT_I_S;

      require(std::abs(headingError(T_W_S.rotation(), T_W_Smeas.rotation())) < 1e-6, "Heading of S does not match the measurement");
      require(T_W_S.translation().isApprox(T_W_Smeas.translation(), 1e-6), "Position of S does not match the measurement");
      if (tiltError.equals(gtsam::Rot3())) {
        require(T_W_S.rotation().equals(R_W_S, 1e-6), "Orientation of S does not match a consistent measurement");
      }
    }
  }
}

// In a fixed frame M, the measurement goes through the guess of T_W_M
void headingInitGoesThroughFixedFrameGuess() {
  const gtsam::Pose3 T_W_M(gtsam::Rot3::Rz(0.7), gtsam::Point3(3.0, 1.0, -0.5));
  const gtsam::Rot3 R_W_S = gtsam::Rot3::Ypr(-1.3, 0.1, 0.3);
  const gtsam::Pose3 T_M_Smeas = T_W_M.inverse() * gtsam::Pose3(R_W_S, gtsam::Point3(-1.0, 4.0, 0.2));
  const gtsam::Pose3 T_W_I = initializeImuPose(R_W_S * kT_I_S.rotation().inverse(), [&](TestEstimator& estimator) {
    estimator.initWorldFrameToFixedFrameTransform(Eigen::Isometry3d(T_W_M.matrix()), "map");
    return estimator.initHeadingAndPosition(poseMeasurement(T_M_Smeas, "map"));
  });

  require((T_W_I * kT_I_S).equals(T_W_M * T_M_Smeas, 1e-6), "Pose of S does not match the measurement through T_W_M");
}

holistic_fusion::UnaryMeasurementXDAbsolute<double, 1> yawMeasurement(const double yaw, const std::string& fixedFrame) {
  return {"yaw",
          10,
          "sensor",
          "sensor_corrected",
          holistic_fusion::RobustNorm::None(),
          1.0,
          1.0,
          yaw,
          Eigen::Matrix<double, 1, 1>::Ones(),
          fixedFrame,
          "world",
          Eigen::Matrix<double, 6, 1>::Ones()};
}

// The yaw of S1 in the world and the position of S2 through T_W_M must match, while roll and pitch stay with gravity
void yawInitMatchesMeasuredYawOfSensor() {
  const gtsam::Pose3 T_W_M(gtsam::Rot3::Ypr(0.3, 0.05, -0.04), gtsam::Point3(3.0, 1.0, -0.5));
  const gtsam::Rot3 R_W_I = gtsam::Rot3::Ypr(-0.9, 0.25, -0.3);
  const double yaw_W_S1meas = 1.2;
  const Eigen::Vector3d M_t_M_S2(0.5, -1.5, 0.3);
  const holistic_fusion::UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3> positionMeasurement(
      "position", 10, "imu", "imu_corrected", holistic_fusion::RobustNorm::None(), 1.0, 1.0, M_t_M_S2, Eigen::Vector3d::Ones(), "map",
      "world", Eigen::Matrix<double, 6, 1>::Ones());
  const gtsam::Pose3 T_W_Iinit = initializeImuPose(R_W_I, [&](TestEstimator& estimator) {
    estimator.initWorldFrameToFixedFrameTransform(Eigen::Isometry3d(T_W_M.matrix()), "map");
    return estimator.initYawAndPosition(yawMeasurement(yaw_W_S1meas, "world"), positionMeasurement);
  });

  const double yaw_W_S1 = (T_W_Iinit.rotation() * kT_I_S.rotation()).yaw();
  require(std::abs(yaw_W_S1 - yaw_W_S1meas) < 1e-6, "Yaw of S1 does not match: " + std::to_string(yaw_W_S1));
  require(std::abs(T_W_Iinit.rotation().pitch() - R_W_I.pitch()) < 1e-6 && std::abs(T_W_Iinit.rotation().roll() - R_W_I.roll()) < 1e-6,
          "Roll and pitch do not stay with gravity");
  require(T_W_Iinit.translation().isApprox(T_W_M.transformFrom(gtsam::Point3(M_t_M_S2)), 1e-6), "Position of S2 does not match");
}

void yawInitRejectsYawInFixedFrame() {
  const holistic_fusion::UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3> positionMeasurement(
      "position", 10, "imu", "imu_corrected", holistic_fusion::RobustNorm::None(), 1.0, 1.0, Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Ones(), "world", "world");
  bool rejected = false;
  try {
    initializeImuPose(gtsam::Rot3(), [&](TestEstimator& estimator) {
      return estimator.initYawAndPosition(yawMeasurement(0.0, "map"), positionMeasurement);
    });
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "A yaw in a fixed frame must be rejected");
}

void initAtStartKeepsTheGravityAlignedStartPose() {
  const gtsam::Rot3 R_W_I = gtsam::Rot3::Ypr(1.1, -0.2, 0.15);
  const gtsam::Pose3 T_W_I = initializeImuPose(R_W_I, [](TestEstimator& estimator) { return estimator.initHeadingAndPositionAtStart(); });

  require(T_W_I.equals(gtsam::Pose3(gtsam::Rot3::Ypr(0.0, -0.2, 0.15), gtsam::Point3::Zero()), 1e-6),
          "Start pose is not gravity-aligned at zero");
}

}  // namespace

int main() {
  try {
    headingInitMatchesMeasuredPoseOfSensor();
    headingInitGoesThroughFixedFrameGuess();
    yawInitMatchesMeasuredYawOfSensor();
    yawInitRejectsYawInFixedFrame();
    initAtStartKeepsTheGravityAlignedStartPose();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
