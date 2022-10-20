// Package
#include "compslam_se/CompslamSeInterface.h"
#include "compslam_se/CompslamSe.h"

namespace compslam_se {

// Public -------------------------------------------------------------------------------------
CompslamSeInterface::CompslamSeInterface() {
  std::cout << YELLOW_START << "CompslamSeInterface" << GREEN_START << " Instance created." << COLOR_END << std::endl;
}

// Protected ------------------------------------------------------------------------------------
bool CompslamSeInterface::setup_() {
  std::cout << YELLOW_START << "CompslamSeInterface" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  if (!graphConfigPtr_ || !staticTransformsPtr_) {
    std::runtime_error("CompslamSeInterface::setup_(): graphConfigPtr_ or staticTransformsPtr_ is not set.");
  }

  compslamSePtr_ = std::make_shared<CompslamSe>();
  compslamSePtr_->setup(graphConfigPtr_, staticTransformsPtr_);

  std::cout << YELLOW_START << "CompslamSeInterface" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
  return true;
}

bool CompslamSeInterface::initYawAndPosition_(const double yaw_W_frame1, const std::string& frame1, const Eigen::Vector3d& W_t_W_frame2,
                                              const std::string& frame2) {
  return compslamSePtr_->initYawAndPosition(yaw_W_frame1, frame1, W_t_W_frame2, frame2);
}

bool CompslamSeInterface::initYawAndPosition_(const Eigen::Matrix4d& T_W_frame, const std::string& frameName) {
  return compslamSePtr_->initYawAndPosition(T_W_frame, frameName);
}

bool CompslamSeInterface::areYawAndPositionInited_() {
  return compslamSePtr_->yawAndPositionInited();
}

void CompslamSeInterface::activateFallbackGraph() {
  compslamSePtr_->activateFallbackGraph();
}

void CompslamSeInterface::addImuMeasurementAndPublishState_(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel,
                                                            const double imuTimeK) {
  static int imuCabinCallbackCounter__ = -1;

  ++imuCabinCallbackCounter__;

  std::shared_ptr<InterfacePrediction> predictionPtr;

  bool success = compslamSePtr_->addImuMeasurement(linearAcc, angularVel, imuTimeK, predictionPtr);

  if (success) {
    publishStateAndMeasureTime_(imuTimeK, predictionPtr->T_W_O, predictionPtr->T_O_Ik, predictionPtr->I_v_W_I, predictionPtr->I_w_W_I);
  }
}

void CompslamSeInterface::addOdometryMeasurement_(const BinaryMeasurement6D& delta) {
  compslamSePtr_->addOdometryMeasurement(delta);
}

void CompslamSeInterface::addUnaryPoseMeasurement_(const UnaryMeasurement6D& unary) {
  compslamSePtr_->addUnaryPoseMeasurement(unary);
}

void CompslamSeInterface::addDualOdometryMeasurement_(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                                                      const Eigen::Matrix<double, 6, 1>& poseBetweenNoise) {
  compslamSePtr_->addDualOdometryMeasurement(odometryKm1, odometryK, poseBetweenNoise);
}

void CompslamSeInterface::addDualGnssPositionMeasurement_(const UnaryMeasurement3D& W_t_W_frame, const Eigen::Vector3d& lastPosition,
                                                          const Eigen::Vector3d& estCovarianceXYZ) {
  compslamSePtr_->addDualGnssPositionMeasurement(W_t_W_frame, lastPosition, estCovarianceXYZ);
}

void CompslamSeInterface::addGnssPositionMeasurement_(const UnaryMeasurement3D& W_t_W_frame) {
  compslamSePtr_->addGnssPositionMeasurement(W_t_W_frame);
}

void CompslamSeInterface::addGnssHeadingMeasurement_(const UnaryMeasurement1D& yaw_W_frame) {
  compslamSePtr_->addGnssHeadingMeasurement(yaw_W_frame);
}

void CompslamSeInterface::publishStateAndMeasureTime_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
                                                      const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I) {
  // Define variables for timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;

  startLoopTime = std::chrono::high_resolution_clock::now();
  publishState_(imuTimeK, T_W_O, T_O_Ik, I_v_W_I, I_w_W_I);
  endLoopTime = std::chrono::high_resolution_clock::now();
  if (std::chrono::duration_cast<std::chrono::microseconds>(endLoopTime - startLoopTime).count() > (1e6 / graphConfigPtr_->imuRate / 2.0)) {
    std::cout << YELLOW_START << "CSe-Interface" << RED_START << " Publishing state took "
              << std::chrono::duration_cast<std::chrono::microseconds>(endLoopTime - startLoopTime).count()
              << " microseconds, which is slower than double IMU-rate (" << (1e6 / graphConfigPtr_->imuRate / 2.0) << " microseconds)."
              << COLOR_END << std::endl;
  }
}

bool CompslamSeInterface::isInNormalOperation() const {
  return compslamSePtr_->getNormalOperationFlag();
}

}  // end namespace compslam_se
