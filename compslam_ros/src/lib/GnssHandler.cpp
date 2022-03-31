// C++
#include <iostream>
// Package
#include "compslam_ros/GnssHandler.h"

namespace m545_estimator {

// Public -------------------------------------------------------------------
GnssHandler::GnssHandler() {
  std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Created Gnss Handler instance." << COLOR_END << std::endl;
}

void GnssHandler::initHandler(const Eigen::Vector3d& accumulatedLeftCoordinates, const Eigen::Vector3d& accumulatedRightCoordinates) {
  std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Initializing the handler." << COLOR_END << std::endl;

  // Initialize GNSS converter
  if (usingGnssReferenceFlag) {
    gnssSensor_.setReference(gnssReferenceLatitude, gnssReferenceLongitude, gnssReferenceAltitude, gnssReferenceHeading);
  } else {
    gnssSensor_.setReference(accumulatedLeftCoordinates(0), accumulatedLeftCoordinates(1), accumulatedLeftCoordinates(2), 0.0);
  }

  // Get Positions
  Eigen::Vector3d leftPosition, rightPosition;
  convertNavSatToPositions(accumulatedLeftCoordinates, accumulatedRightCoordinates, leftPosition, rightPosition);

  // Get heading (assuming that connection between antennas is perpendicular to heading)
  Eigen::Vector3d W_t_heading = getRobotHeading_(leftPosition, rightPosition);
  std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Heading read from the GNSS is the following: " << W_t_heading << std::endl;

  // Get initial global yaw
  globalAttitudeYaw_ = computeYawFromHeadingVector_(W_t_heading);
  std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Initial global yaw is: " << 180 / M_PI * globalAttitudeYaw_ << std::endl;

  // Initial GNSS position
  W_t_W_GnssL0_ = leftPosition;
}

void GnssHandler::convertNavSatToPositions(const Eigen::Vector3d& leftGnssCoordinate, const Eigen::Vector3d& rightGnssCoordinate,
                                           Eigen::Vector3d& leftPosition, Eigen::Vector3d& rightPosition) {
  /// Left
  leftPosition = gnssSensor_.gpsToCartesian(leftGnssCoordinate(0), leftGnssCoordinate(1), leftGnssCoordinate(2));

  /// Right
  rightPosition = gnssSensor_.gpsToCartesian(rightGnssCoordinate(0), rightGnssCoordinate(1), rightGnssCoordinate(2));
}

double GnssHandler::getYawFromPositions(const Eigen::Vector3d& leftPosition, const Eigen::Vector3d& rightPosition) {
  Eigen::Vector3d robotHeading = getRobotHeading_(leftPosition, rightPosition);
  return computeYawFromHeadingVector_(robotHeading);
}

// Private ------------------------------------------------------------------
Eigen::Vector3d GnssHandler::getRobotHeading_(const Eigen::Vector3d& leftPosition, const Eigen::Vector3d& rightPosition) {
  // Compute connecting unity vector
  Eigen::Vector3d W_t_GnssR_GnssL = (leftPosition - rightPosition).normalized();

  // Compute forward pointing vector
  Eigen::Vector3d zUnityVector(0.0, 0.0, -1.0);
  Eigen::Vector3d W_t_heading = zUnityVector.cross(W_t_GnssR_GnssL).normalized();

  return W_t_heading;
}

double GnssHandler::computeYawFromHeadingVector_(const Eigen::Vector3d& headingVector) {
  double yaw = atan2(headingVector(1), headingVector(0));
  // Compute angle
  if (yaw > M_PI) {
    return yaw - (2 * M_PI);
  } else if (yaw < -M_PI) {
    return yaw + (2 * M_PI);
  } else {
    return yaw;
  }
}

}  // namespace m545_estimator
