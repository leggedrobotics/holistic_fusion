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
  compslamSePtr_->setup(node, privateNode, staticTransformsPtr_);

  std::cout << YELLOW_START << "CompslamSeInterface" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
  return true;
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

void CompslamSeInterface::addOdometryMeasurement_(const Eigen::Matrix4d& T_O_Lk, const ros::Time& odometryTimeK) {
  compslamSePtr_->addOdometryMeasurement(T_O_Lk, odometryTimeK);
}

void CompslamSeInterface::addGnssMeasurements_(const Eigen::Vector3d& leftGnssCoord, const Eigen::Vector3d& rightGnssCoord,
                                               const Eigen::Vector3d& covarianceXYZ, const ros::Time& gnssTimeK) {
  compslamSePtr_->addGnssMeasurements(leftGnssCoord, rightGnssCoord, covarianceXYZ, gnssTimeK);
}

}  // end namespace compslam_se