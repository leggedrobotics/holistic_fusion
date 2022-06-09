#ifndef COMPSLAM_SE_INTERFACE_H
#define COMPSLAM_SE_INTERFACE_H

#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Eigen>
#include <thread>

// Workspace
#include "StaticTransforms.h"
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
  bool setup_(ros::NodeHandle& node);

  // Required for initialization
  bool initYawAndPosition_(const double yaw_W_frame1, const std::string& frame1, const Eigen::Vector3d& t_W_frame2,
                           const std::string& frame2);
  bool initYawAndPosition_(Eigen::Matrix4d T_O_I);
  bool areYawAndPositionInited_();

  // Graph Maniupulation

  void activateFallbackGraph();

  // Write measurements
  void addImuMeasurement_(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK);
  void addOdometryMeasurement_(const DeltaMeasurement6D& delta);
  void addUnaryPoseMeasurement_(const UnaryMeasurement6D& unary);
  void addOdometryMeasurement_(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                               const Eigen::Matrix<double, 6, 1>& poseBetweenNoise);
  void addGnssPositionMeasurement_(const Eigen::Vector3d& position, const Eigen::Vector3d& lastPosition,
                                   const Eigen::Vector3d& covarianceXYZ, const double gnssTimeK, const double rate,
                                   const double positionUnaryNoise);
  void addGnssHeadingMeasurement_(const double yaw_W_frame, const std::string& frameName, const double gnssTimeK, const double rate,
                                  const double yawUnaryNoise);

  // Publish
  virtual void publishState_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
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

 private:
  // Threads
  std::thread publishStateThread_;

  // Functions
  void publishStateAndMeasureTime_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
                                   const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I);
};

}  // namespace compslam_se

#endif  // COMPSLAM_STATEESTIMATION_INTERFACE_H
