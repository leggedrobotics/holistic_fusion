#ifndef GNSSHANDLER_H
#define GNSSHANDLER_H

// C++
#include <Eigen/Eigen>
// Workspace
#include "robot_utils/sensors/GNSS.hpp"

// Defined macros
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

namespace compslam_se {

class GnssHandler {
 public:
  GnssHandler();

  // Methods
  void initHandler(const Eigen::Vector3d& accumulatedLeftCoordinates__, const Eigen::Vector3d& accumulatedRightCoordinates);

  void convertNavSatToPositions(const Eigen::Vector3d& leftGnssCoordinate, const Eigen::Vector3d& rightGnssCoordinate,
                                Eigen::Vector3d& leftPosition, Eigen::Vector3d& rightPosition);
  double getYawFromPositions(const Eigen::Vector3d& leftPosition, const Eigen::Vector3d& rightPosition);

  // Flags
  bool usingGnssReferenceFlag = false;

  // Member Variables
  double gnssReferenceLatitude;
  double gnssReferenceLongitude;
  double gnssReferenceAltitude;
  double gnssReferenceHeading;

 private:
  // Member methods
  Eigen::Vector3d getRobotHeading_(const Eigen::Vector3d& leftPosition, const Eigen::Vector3d& rightPosition);
  //// Compute yaw from the heading vector
  static double computeYawFromHeadingVector_(const Eigen::Vector3d& headingVector);

  // Member variables
  robot_utils::GNSS gnssSensor_;
  Eigen::Vector3d W_t_W_GnssL0_;
  double globalAttitudeYaw_;
};

}  // namespace compslam_se

#endif  // GNSSHANDLER_H
