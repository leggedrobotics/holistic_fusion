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

  compslamSePtr_ = new CompslamSe();
  compslamSePtr_->setup(graphConfigPtr_, staticTransformsPtr_);

  std::cout << YELLOW_START << "CompslamSeInterface" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
  return true;
}

bool CompslamSeInterface::initYawAndPosition_(const double yaw_W_frame1, const std::string& frame1, const Eigen::Vector3d& t_W_frame2,
                                              const std::string& frame2) {
  return compslamSePtr_->initYawAndPosition(yaw_W_frame1, frame1, t_W_frame2, frame2);
}

bool CompslamSeInterface::initYawAndPosition_(Eigen::Matrix4d T_O_I) {
  return compslamSePtr_->initYawAndPosition(T_O_I);
}

bool CompslamSeInterface::areYawAndPositionInited_() {
  return compslamSePtr_->yawAndPositionInited();
}

void CompslamSeInterface::activateFallbackGraph() {
  compslamSePtr_->activateFallbackGraph();
}

/** \brief Rotate the given vector by the specified angles around the z-, x- respectively y-axis.
 *
 * @param v the vector to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
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

void CompslamSeInterface::addOdometryMeasurement_(const DeltaMeasurement6D& delta) {
  compslamSePtr_->addOdometryMeasurement(delta);
}

void CompslamSeInterface::addUnaryPoseMeasurement_(const UnaryMeasurement6D& unary) {
  compslamSePtr_->addUnaryPoseMeasurement(unary);
}

void CompslamSeInterface::addOdometryMeasurement_(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                                                  const Eigen::Matrix<double, 6, 1>& poseBetweenNoise) {
  compslamSePtr_->addOdometryMeasurement(odometryKm1, odometryK, poseBetweenNoise);
}

void CompslamSeInterface::addGnssPositionMeasurement_(const Eigen::Vector3d& position, const Eigen::Vector3d& lastPosition,
                                                      const Eigen::Vector3d& covarianceXYZ, const double gnssTimeK, const double rate,
                                                      const double positionUnaryNoise) {
  compslamSePtr_->addGnssPositionMeasurement(position, lastPosition, covarianceXYZ, gnssTimeK, rate, positionUnaryNoise);
}

void CompslamSeInterface::addGnssHeadingMeasurement_(const double yaw_W_frame, const std::string& frameName, const double gnssTimeK,
                                                     const double rate, const double yawUnaryNoise) {
  compslamSePtr_->addGnssHeadingMeasurement(yaw_W_frame, frameName, gnssTimeK, rate, yawUnaryNoise);
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

}  // end namespace compslam_se