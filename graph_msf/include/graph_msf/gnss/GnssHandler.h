/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_GNSS_HANDLER_H
#define GRAPH_MSF_GNSS_HANDLER_H

// C++
#include <Eigen/Eigen>

// Workspace
#include "graph_msf/gnss/Gnss.h"
#include "graph_msf/interface/Terminal.h"

namespace graph_msf {

class GnssHandler {
 public:
  GnssHandler();

  // Methods
  void initHandler(const Eigen::Vector3d& accumulatedLeftCoordinates, const Eigen::Vector3d& accumulatedRightCoordinates);
  void initHandler(const Eigen::Vector3d& accumulatedCoordinates);

  void convertNavSatToPositions(const Eigen::Vector3d& leftGnssCoordinate, const Eigen::Vector3d& rightGnssCoordinate,
                                Eigen::Vector3d& leftPosition, Eigen::Vector3d& rightPosition);
  void convertNavSatToPosition(const Eigen::Vector3d& gnssCoordinate, Eigen::Vector3d& position);
  double computeYaw(const Eigen::Vector3d& gnssPos1, const Eigen::Vector3d& gnssPos2);

  // Flags
  bool usingGnssReferenceFlag = false;

  // Setters
  void setGnssReferenceLatitude(const double gnssReferenceLatitude) { gnssReferenceLatitude_ = gnssReferenceLatitude; }
  void setGnssReferenceLongitude(const double gnssReferenceLongitude) { gnssReferenceLongitude_ = gnssReferenceLongitude; }
  void setGnssReferenceAltitude(const double gnssReferenceAltitude) { gnssReferenceAltitude_ = gnssReferenceAltitude; }
  void setGnssReferenceHeading(const double gnssReferenceHeading) { gnssReferenceHeading_ = gnssReferenceHeading; }

  // State Machine based bookkeeping.
  double globalYawDegFromFile_{0.0};
  bool useYawInitialGuessFromFile_{false};
  bool yawInitialGuessFromAlignment_{false};

 private:
  // Member methods
  Eigen::Vector3d computeHeading_(const Eigen::Vector3d& gnssPos1, const Eigen::Vector3d& gnssPos2);
  //// Compute yaw from the heading vector
  static double computeYawFromHeadingVector_(const Eigen::Vector3d& headingVector);

  // Member variables
  Gnss gnssSensor_;
  Eigen::Vector3d W_t_W_GnssL0_;
  double globalAttitudeYaw_;

  // Reference Parameters
  double gnssReferenceLatitude_;
  double gnssReferenceLongitude_;
  double gnssReferenceAltitude_;
  double gnssReferenceHeading_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_GNSS_HANDLER_H
