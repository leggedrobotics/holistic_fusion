#ifndef COMPSLAM_SE_INTERFACE_H
#define COMPSLAM_SE_INTERFACE_H

#include <ros/time.h>
#include <Eigen/Eigen>

// Workspace
#include "compslam_se/StaticTransforms.h"
#include "compslam_se/config/GraphConfig.h"

namespace compslam_se {

class CompslamSe;

class CompslamSeInterface {
 public:
  CompslamSeInterface();

 protected:
  // Setup
  bool setup_(ros::NodeHandle& node, ros::NodeHandle& privateNode);

  // Write measurements
  void addImuMeasurement_(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const ros::Time& imuTimeK);
  void addOdometryMeasurement_(const Eigen::Matrix4d& T_O_Lk, const double rate, std::vector<double> poseBetweenNoise,
                               const ros::Time& odometryTimeK);
  void addGnssMeasurements_(const Eigen::Vector3d& leftGnssCoord, const Eigen::Vector3d& rightGnssCoord,
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
  /// Static transforms
  StaticTransforms* staticTransformsPtr_ = NULL;
  /// Graph Configuration
  GraphConfig* graphConfigPtr_ = NULL;
  /// Verbosity
  int verboseLevel_ = 0;
  /// Logging
  bool logPlots_ = false;

  /// Flags
  bool usingGnssReferenceFlag_ = false;
  bool usingGnssFlag_ = true;
  bool usingCompslamFlag_ = true;
  /// GNSS Reference
  double gnssReferenceLatitude_ = 0.0;
  double gnssReferenceLongitude_ = 0.0;
  double gnssReferenceAltitude_ = 0.0;
  double gnssReferenceHeading_ = 0.0;
};

}  // namespace compslam_se

#endif  // COMPSLAM_STATEESTIMATION_INTERFACE_H
