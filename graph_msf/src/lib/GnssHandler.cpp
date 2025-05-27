/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <iostream>
// Package
#include "graph_msf/gnss/GnssHandler.h"

namespace graph_msf {

// Public -------------------------------------------------------------------
GnssHandler::GnssHandler() {
  std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Created Gnss Handler instance." << COLOR_END << std::endl;
}

void GnssHandler::initHandler(const Eigen::Vector3d& accumulatedLeftCoordinates, const Eigen::Vector3d& accumulatedRightCoordinates) {
  std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Initializing the handler." << COLOR_END << std::endl;

  // Initialize Gnss converter
  if (useGnssReferenceFlag_) {
    std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Setting pre-defined reference." << COLOR_END << std::endl;
    gnssSensor_.setReference(presetGnssReferenceLatitude_, presetGnssReferenceLongitude_, presetGnssReferenceAltitude_,
                             presetGnssReferenceHeading_);
  } else {
    std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Setting coordinate origin to current position." << COLOR_END
              << std::endl;
    gnssSensor_.setReference(accumulatedLeftCoordinates(0), accumulatedLeftCoordinates(1), accumulatedLeftCoordinates(2), 0.0);
  }

  // Get Positions
  Eigen::Vector3d leftPosition, rightPosition;
  convertNavSatToPositions(accumulatedLeftCoordinates, accumulatedRightCoordinates, leftPosition, rightPosition);

  // Get heading (assuming that connection between antennas is perpendicular to heading)
  Eigen::Vector3d W_t_heading = computeHeading_(leftPosition, rightPosition);
  std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Heading read from the Gnss is the following: " << W_t_heading << std::endl;

  // Get initial global yaw
  globalAttitudeYaw_ = computeYawFromHeadingVector_(W_t_heading);
  std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Initial global yaw is: " << 180 / M_PI * globalAttitudeYaw_ << std::endl;
}

void GnssHandler::initHandler(const Eigen::Vector3d& accumulatedCoordinates) {
  std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Initializing the handler position." << COLOR_END << std::endl;

  // Initialize Gnss converter
  if (useGnssReferenceFlag_) {
    std::cout << YELLOW_START << "GnssHandler" << GREEN_START
              << " Setting GNSS reference frame coordinate origin to pre-defined reference." << COLOR_END << std::endl;
    gnssSensor_.setReference(presetGnssReferenceLatitude_, presetGnssReferenceLongitude_, presetGnssReferenceAltitude_,
                             presetGnssReferenceHeading_);
    if (useSicilianEnu_) {
      sicilianEnu_.setAnchor(presetGnssReferenceLatitude_, presetGnssReferenceLongitude_, presetGnssReferenceAltitude_);
    }
  } else {
    std::cout << YELLOW_START << "GnssHandler" << GREEN_START
              << " Setting GNSS reference frame coordinate origin to current position." << COLOR_END << std::endl;
    gnssSensor_.setReference(accumulatedCoordinates(0), accumulatedCoordinates(1), accumulatedCoordinates(2), 0.0);
    std::cout << YELLOW_START << "GnssHandler" << GREEN_START << " Reference: " << accumulatedCoordinates.transpose() << COLOR_END
              << std::endl;
    if (useSicilianEnu_) {
      std::cout << "\n\n\033[1;95m************************************************************\n"
           "*                  GnssHandler                             *\n"
           "*        Setting Sicilian ENU anchor to current position    *\n"
           "************************************************************\033[0m\n\n" << std::endl;
      sicilianEnu_.setAnchor(accumulatedCoordinates(0), accumulatedCoordinates(1), accumulatedCoordinates(2));
    }
  }
}

void GnssHandler::convertNavSatToPositions(const Eigen::Vector3d& leftGnssCoordinate, const Eigen::Vector3d& rightGnssCoordinate,
                                           Eigen::Vector3d& leftPosition, Eigen::Vector3d& rightPosition) {
  /// Left
  leftPosition = gnssSensor_.gnssToCartesian(leftGnssCoordinate(0), leftGnssCoordinate(1), leftGnssCoordinate(2));

  /// Right
  rightPosition = gnssSensor_.gnssToCartesian(rightGnssCoordinate(0), rightGnssCoordinate(1), rightGnssCoordinate(2));
}

void GnssHandler::convertNavSatToPosition(const Eigen::Vector3d& gnssCoordinate, Eigen::Vector3d& position) {
  if (useSicilianEnu_ && sicilianEnuReady_) {
    // Use SicilianENU conversion
    SicilianENU::Result res = sicilianEnu_.forward(gnssCoordinate(0), gnssCoordinate(1), gnssCoordinate(2));
    position = Eigen::Vector3d(res.x, res.y, res.zEllip);
  } else {
    std::cout << RED_START << "GnssHandler" << RED_START
          << " ERROR: SicilianENU is not ready but a conversion was requested!" << COLOR_END << std::endl;
    // Use legacy Gnss
    position = gnssSensor_.gnssToCartesian(gnssCoordinate(0), gnssCoordinate(1), gnssCoordinate(2));
  }
}

// Heading is defined as the orthogonal vector pointing from gnssPos2 to gnssPos1, projected to x,y-plane
// Hence, if left and right gnss, then gnssPos1=leftGnss, gnssPos2=rightGnss
double GnssHandler::computeYaw(const Eigen::Vector3d& gnssPos1, const Eigen::Vector3d& gnssPos2) {
  Eigen::Vector3d robotHeading = computeHeading_(gnssPos1, gnssPos2);
  return computeYawFromHeadingVector_(robotHeading);
}

void GnssHandler::convertNavSatToPositionLV03(const Eigen::Vector3d& gnssCoordinate, Eigen::Vector3d& position) {
  position = gnssSensor_.gnssToLv03(gnssCoordinate(0), gnssCoordinate(1), gnssCoordinate(2));
}

// Private ------------------------------------------------------------------

// Heading is defined as the orthogonal vector pointing from gnssPos2 to gnssPos1, projected to x,y-plane
Eigen::Vector3d GnssHandler::computeHeading_(const Eigen::Vector3d& gnssPos1, const Eigen::Vector3d& gnssPos2) {
  // Compute connecting unity vector
  Eigen::Vector3d W_t_GnssR_GnssL = (gnssPos1 - gnssPos2).normalized();

  // Compute forward pointing vector
  Eigen::Vector3d zUnityVector = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d W_t_heading = W_t_GnssR_GnssL.cross(zUnityVector).normalized();

  return W_t_heading;
}

double GnssHandler::computeYawFromHeadingVector_(const Eigen::Vector3d& headingVector) {
  return atan2(headingVector(1), headingVector(0));
}

}  // namespace graph_msf
