#ifndef COMPSLAM_SE_INTERFACE_H
#define COMPSLAM_SE_INTERFACE_H

#include <Eigen/Eigen>
#include <thread>

// Workspace
#include "StaticTransforms.h"
#include "compslam_se/config/GraphConfig.h"
#include "compslam_se/measurements/BinaryMeasurement6D.h"
#include "compslam_se/measurements/UnaryMeasurement1D.h"
#include "compslam_se/measurements/UnaryMeasurement3D.h"
#include "compslam_se/measurements/UnaryMeasurement6D.h"

namespace compslam_se {

class CompslamSe;

class CompslamSeInterface {
 public:
  CompslamSeInterface();

 protected:
  // Setup
  bool setup_();

  // Required for initialization
  bool initYawAndPosition_(const double yaw_W_frame1, const std::string& frame1, const Eigen::Vector3d& t_W_frame2,
                           const std::string& frame2);
  bool initYawAndPosition_(const Eigen::Matrix4d& T_W_frame, const std::string& frameName);
  bool areYawAndPositionInited_();

  // Graph Maniupulation

  void activateFallbackGraph();

  // Write measurements
  void addImuMeasurementAndPublishState_(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK);
  void addOdometryMeasurement_(const BinaryMeasurement6D& delta);
  void addUnaryPoseMeasurement_(const UnaryMeasurement6D& unary);
  void addDualOdometryMeasurement_(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                                   const Eigen::Matrix<double, 6, 1>& poseBetweenNoise);
  void addDualGnssPositionMeasurement_(const UnaryMeasurement3D& W_t_W_frame, const Eigen::Vector3d& lastPosition,
                                       const Eigen::Vector3d& estCovarianceXYZ);
  void addGnssPositionMeasurement_(const UnaryMeasurement3D& W_t_W_frame);
  void addGnssHeadingMeasurement_(const UnaryMeasurement1D& yaw_W_frame);

  // Publish
  virtual void publishState_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
                             const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I) = 0;

  // Getters
  bool isInNormalOperation() const;

  // Member Variables
  /// CompslamSe
  std::shared_ptr<CompslamSe> compslamSePtr_ = NULL;
  /// Graph Configuration
  std::shared_ptr<GraphConfig> graphConfigPtr_ = NULL;
  std::shared_ptr<StaticTransforms> staticTransformsPtr_ = NULL;

  /// Verbosity
  int verboseLevel_ = 0;

 private:
  // Threads
  std::thread publishStateThread_;

  // Functions
  void publishStateAndMeasureTime_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
                                   const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I);
};

}  // namespace compslam_se

#endif  // COMPSLAM_STATEESTIMATION_INTERFACE_H
