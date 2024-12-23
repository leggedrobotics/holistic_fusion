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
  // Init
  void initHandler(const Eigen::Vector3d& accumulatedLeftCoordinates, const Eigen::Vector3d& accumulatedRightCoordinates);
  void initHandler(const Eigen::Vector3d& accumulatedCoordinates);

  // Conversion methods
  void convertNavSatToPositions(const Eigen::Vector3d& leftGnssCoordinate, const Eigen::Vector3d& rightGnssCoordinate,
                                Eigen::Vector3d& leftPosition, Eigen::Vector3d& rightPosition);
  void convertNavSatToPosition(const Eigen::Vector3d& gnssCoordinate, Eigen::Vector3d& position);
  double computeYaw(const Eigen::Vector3d& gnssPos1, const Eigen::Vector3d& gnssPos2);
  void convertNavSatToPositionLV03(const Eigen::Vector3d& gnssCoordinate, Eigen::Vector3d& position);

  // Setters
  // For state machine and bookkeeping
  void setUseGnssReferenceFlag(const bool useGnssReferenceFlag) { useGnssReferenceFlag_ = useGnssReferenceFlag; }
  void setGlobalYawDegFromFile(const double globalYawDegFromFile) { globalYawDegFromFile_ = globalYawDegFromFile; }
  void setUseYawInitialGuessFromFile(const bool useYawInitialGuessFromFile) { useYawInitialGuessFromFile_ = useYawInitialGuessFromFile; }
  void setUseYawInitialGuessFromAlignment(const bool useYawInitialGuessFromAlignment) {
    useYawInitialGuessFromAlignment_ = useYawInitialGuessFromAlignment;
  }
  // Actual Reference Coordinate Parameters
  void setGnssReferenceLatitude(const double gnssReferenceLatitude) { presetGnssReferenceLatitude_ = gnssReferenceLatitude; }
  void setGnssReferenceLongitude(const double gnssReferenceLongitude) { presetGnssReferenceLongitude_ = gnssReferenceLongitude; }
  void setGnssReferenceAltitude(const double gnssReferenceAltitude) { presetGnssReferenceAltitude_ = gnssReferenceAltitude; }
  void setGnssReferenceHeading(const double gnssReferenceHeading) { presetGnssReferenceHeading_ = gnssReferenceHeading; }

  // Getters
  bool getUseGnssReferenceFlag() const { return useGnssReferenceFlag_; }
  double getGlobalYawDegFromFile() const { return globalYawDegFromFile_; }
  bool getUseYawInitialGuessFromFile() const { return useYawInitialGuessFromFile_; }
  bool getUseYawInitialGuessFromAlignment() const { return useYawInitialGuessFromAlignment_; }
  // Reference Coordinate Parameters
  double getGnssReferenceLatitude() const { return gnssSensor_.getReferenceLatitude(); }
  double getGnssReferenceLongitude() const { return gnssSensor_.getReferenceLongitude(); }
  double getGnssReferenceAltitude() const { return gnssSensor_.getReferenceAltitude(); }
  double getGnssReferenceHeading() const { return gnssSensor_.getReferenceHeading(); }

 private:
  // Member methods
  Eigen::Vector3d computeHeading_(const Eigen::Vector3d& gnssPos1, const Eigen::Vector3d& gnssPos2);
  //// Compute yaw from the heading vector
  static double computeYawFromHeadingVector_(const Eigen::Vector3d& headingVector);

  // Member variables
  Gnss gnssSensor_;
  double globalAttitudeYaw_;

  // State Machine and bookkeeping.
  bool useGnssReferenceFlag_ = false;
  double globalYawDegFromFile_{0.0};
  bool useYawInitialGuessFromFile_{false};
  bool useYawInitialGuessFromAlignment_{false};

  // Reference Coordinate Parameters
  double presetGnssReferenceLatitude_;
  double presetGnssReferenceLongitude_;
  double presetGnssReferenceAltitude_;
  double presetGnssReferenceHeading_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_GNSS_HANDLER_H
