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

void CompslamSeInterface::addImuMeasurement_(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel,
                                             const ros::Time& imuTimeK) {
  InterfacePrediction* predictionPtr;
  bool success = compslamSePtr_->addImuMeasurement(linearAcc, angularVel, imuTimeK, predictionPtr);

  if (success) {
    publishState_(imuTimeK, predictionPtr->T_W_O, predictionPtr->T_O_Ik, predictionPtr->I_v_W_I, predictionPtr->I_w_W_I);
    delete predictionPtr;
  }
}

void CompslamSeInterface::addOdometryMeasurement_(const Eigen::Matrix4d& T_O_Lk, const double rate, std::vector<double> poseBetweenNoise,
                                                  const ros::Time& odometryTimeK) {
  compslamSePtr_->addOdometryMeasurement(T_O_Lk, rate, poseBetweenNoise, odometryTimeK);
}

void CompslamSeInterface::addGnssPositionMeasurement_(const Eigen::Vector3d& position, const Eigen::Vector3d& lastPosition,
                                                      const Eigen::Vector3d& covarianceXYZ, const ros::Time& gnssTimeK, const double rate,
                                                      const double positionUnaryNoise) {
  compslamSePtr_->addGnssPositionMeasurement(position, lastPosition, covarianceXYZ, gnssTimeK, rate, positionUnaryNoise);
}

void CompslamSeInterface::addGnssHeadingMeasurement_(const double heading, const Eigen::Vector3d& covarianceXYZ, const ros::Time& gnssTimeK,
                                                     const double rate, const double positionUnaryNoise) {
  compslamSePtr_->addGnssHeadingMeasurement(heading, covarianceXYZ, gnssTimeK, rate, positionUnaryNoise);
}

// wheel odometry measurement must be in cabin frame
void CompslamSeInterface::addWheelOdometryMeasurement_(const ros::Time& woTimeK, const double rate, const std::vector<double>& woSpeedNoise,
                                                       const Eigen::Vector3d& linearVel, const Eigen::Vector3d& angularVel) {

  compslamSePtr_->addWheelOdometryMeasurement(woTimeK, rate, woSpeedNoise, linearVel, angularVel);
}


}  // end namespace compslam_se