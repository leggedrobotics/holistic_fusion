#ifndef COMPSLAM_SE_INTERFACE_H
#define COMPSLAM_SE_INTERFACE_H

#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Eigen>

// Workspace
#include "compslam_se/StaticTransforms.h"
#include "compslam_se/config/GraphConfig.h"
#include "compslam_se/measurements/DeltaMeasurement6D.h"
#include "compslam_se/measurements/UnaryMeasurement6D.h"

namespace compslam_se {

class CompslamSe;

class CompslamSeInterface {
 public:
  CompslamSeInterface();

 protected:
  // Setup
  bool setup_(ros::NodeHandle& node, ros::NodeHandle& privateNode);

  // Required for initialization
  bool initYawAndPosition_(const double yaw, const Eigen::Vector3d& position);
  bool areYawAndPositionInited_();

  // Graph Maniupulation
  void activateFallbackGraph();

  // Write measurements
  void addImuMeasurement_(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const ros::Time& imuTimeK);
  void addOdometryMeasurement_(const DeltaMeasurement6D& delta);
  void addOdometryMeasurement_(const UnaryMeasurement6D& unary);
  void addOdometryMeasurement_(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                               const Eigen::Matrix<double, 6, 1>& poseBetweenNoise);
  void addGnssPositionMeasurement_(const Eigen::Vector3d& position, const Eigen::Vector3d& lastPosition,
                                   const Eigen::Vector3d& covarianceXYZ, const ros::Time& gnssTimeK, const double rate,
                                   const double positionUnaryNoise);

  // Publish
  virtual void publishState_(ros::Time imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
                             const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I) = 0;

  // Get Parameters
  virtual void readParams_(const ros::NodeHandle& privateNode) = 0;

  // Data Manipulation

  // Member Variables
  /// CompslamSe
  CompslamSe* compslamSePtr_ = NULL;
  /// Graph Configuration
  GraphConfig* graphConfigPtr_ = NULL;
  StaticTransforms* staticTransformsPtr_ = NULL;
  /// Verbosity
  int verboseLevel_ = 0;
  /// Logging
  bool logPlots_ = false;
};

}  // namespace compslam_se

#endif  // COMPSLAM_STATEESTIMATION_INTERFACE_H
