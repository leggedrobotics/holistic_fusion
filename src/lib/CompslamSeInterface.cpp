// Package
#include "compslam_se/CompslamSeInterface.h"
#include "compslam_se/CompslamSe.h"

namespace compslam_se {

// Public -------------------------------------------------------------------------------------
CompslamSeInterface::CompslamSeInterface() {
  std::cout << YELLOW_START << "CompslamSeInterface" << GREEN_START << " Instance created." << COLOR_END << std::endl;
}

// Protected ------------------------------------------------------------------------------------
bool CompslamSeInterface::setup_(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  std::cout << YELLOW_START << "CompslamSeInterface" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  compslamSePtr_ = new CompslamSe();
  compslamSePtr_->setup(node, privateNode, graphConfigPtr_, staticTransformsPtr_);

  std::cout << YELLOW_START << "CompslamSeInterface" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
  return true;
}

bool CompslamSeInterface::initYawAndPosition_(const double yaw, const Eigen::Vector3d& position) {
  return compslamSePtr_->initYawAndPosition(yaw, position);
}

bool CompslamSeInterface::areYawAndPositionInited_() {
  return compslamSePtr_->areYawAndPositionInited();
}

void CompslamSeInterface::activateFallbackGraph() {
  compslamSePtr_->activateFallbackGraph();
}

void CompslamSeInterface::addImuMeasurement_(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel,
                                             const ros::Time& imuTimeK) {
  InterfacePrediction* predictionPtr;
  bool success = compslamSePtr_->addImuMeasurement(linearAcc, angularVel, imuTimeK, predictionPtr);

  if (success) {
    publishState_(imuTimeK, predictionPtr->T_W_O, predictionPtr->T_O_Ik, predictionPtr->I_v_W_I, predictionPtr->I_w_W_I);
    delete predictionPtr;
  }
}

void CompslamSeInterface::addOdometryMeasurement_(const DeltaMeasurement6D& delta) {
  compslamSePtr_->addOdometryMeasurement(delta);
}

void CompslamSeInterface::addOdometryMeasurement_(const UnaryMeasurement6D& unary) {
  compslamSePtr_->addOdometryMeasurement(unary);
}

void CompslamSeInterface::addOdometryMeasurement_(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                                                  const Eigen::Matrix<double, 6, 1>& poseBetweenNoise) {
  compslamSePtr_->addOdometryMeasurement(odometryKm1, odometryK, poseBetweenNoise);
}

void CompslamSeInterface::addGnssPositionMeasurement_(const Eigen::Vector3d& position, const Eigen::Vector3d& lastPosition,
                                                      const Eigen::Vector3d& covarianceXYZ, const ros::Time& gnssTimeK, const double rate,
                                                      const double positionUnaryNoise) {
  compslamSePtr_->addGnssPositionMeasurement(position, lastPosition, covarianceXYZ, gnssTimeK, rate, positionUnaryNoise);
}

void CompslamSeInterface::addGnssHeadingMeasurement_(const double yaw, const double gnssTimeK, const double rate,
                                                     const double yawUnaryNoise) {
  compslamSePtr_->addGnssHeadingMeasurement(yaw, gnssTimeK, rate, yawUnaryNoise);
}

}  // end namespace compslam_se