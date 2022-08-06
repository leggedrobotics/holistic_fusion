#ifndef GnssHandler_H
#define GnssHandler_H

// C++
#include <Eigen/Eigen>
// Workspace
#include "compslam_se/gnss/Gnss.h"

// Defined macros
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

namespace compslam_se {

class GnssHandler {
 public:
  GnssHandler();

  // Methods
  void initHandler(const Eigen::Vector3d& accumulatedLeftCoordinates, const Eigen::Vector3d& accumulatedRightCoordinates);

  void convertNavSatToPositions(const Eigen::Vector3d& leftGnssCoordinate, const Eigen::Vector3d& rightGnssCoordinate,
                                Eigen::Vector3d& leftPosition, Eigen::Vector3d& rightPosition);
  double computeYaw(const Eigen::Vector3d& gnssPos1, const Eigen::Vector3d& gnssPos2);

  // Flags
  bool usingGnssReferenceFlag = false;

  // Setters
  void setGnssReferenceLatitude(const double gnssReferenceLatitude) { gnssReferenceLatitude_ = gnssReferenceLatitude; }
  void setGnssReferenceLongitude(const double gnssReferenceLongitude) { gnssReferenceLongitude_ = gnssReferenceLongitude; }
  void setGnssReferenceAltitude(const double gnssReferenceAltitude) { gnssReferenceAltitude_ = gnssReferenceAltitude; }
  void setGnssReferenceHeading(const double gnssReferenceHeading) { gnssReferenceHeading_ = gnssReferenceHeading; }

 private:
  // Member methods
  Eigen::Vector3d computeHeading_(const Eigen::Vector3d& gnssPos1, const Eigen::Vector3d& gnssPos2);
  //// Compute yaw from the heading vector
  static double computeYawFromHeadingVector_(const Eigen::Vector3d& headingVector);

  // Member variables
  compslam_se::Gnss gnssSensor_;
  Eigen::Vector3d W_t_W_GnssL0_;
  double globalAttitudeYaw_;

  // Reference Parameters
  double gnssReferenceLatitude_;
  double gnssReferenceLongitude_;
  double gnssReferenceAltitude_;
  double gnssReferenceHeading_;
};

}  // namespace compslam_se

#endif  // GnssHandler_H
