#ifndef GRAPH_MSF_GNSS_H_
#define GRAPH_MSF_GNSS_H_

#pragma once

#include <Eigen/Dense>

namespace graph_msf {

class Gnss {
 public:
  Gnss();
  ~Gnss() = default;

  // Setters
  void setReference(const double& referenceLatitude, const double& referenceLongitude, const double& referenceAltitude,
                    const double& referenceHeading);
  void setMercatorReferenceFrame(const Eigen::Vector3d& newReferencePoint);

  // Conversions
  Eigen::Vector3d gnssToCartesian(const double& latitudeInDegrees, const double& longitudeInDegrees, const double& altitude);

  Eigen::Vector3d cartesianToGps(const Eigen::Ref<const Eigen::Vector3d>& position) const;

  Eigen::Vector3d besselEllipsoidToMercator(const double& latitudeInRad, const double& longitudeInRad, const double& altitude);

  Eigen::Vector3d gnssToLv03Raw(const double latitudeInDegrees, const double longitudeInDegrees, const double altitude);

  Eigen::Vector3d gnssToLv03(const double latitudeInDegrees, const double longitudeInDegrees, const double altitude);

  // Utility
  double wGStoCHx(double lat, double lon);
  double wGStoCHy(double lat, double lon);
  double decToSexAngle(double dec);
  double sexAngleToSeconds(double dms);

  // Getters
  double getReferenceLongitude() const { return referenceLongitude_; }
  double getReferenceLatitude() const { return referenceLatitude_; }
  double getReferenceAltitude() const { return referenceAltitude_; }
  double getReferenceHeading() const { return referenceHeading_; }

 protected:
  void calculateConversionParameters();

  double referenceLongitude_ = 7.43958;
  double referenceLatitude_ = 46.95241;
  double referenceAltitude_ = 0.0;
  double referenceHeading_ = 0.0;

  Eigen::Vector3d referencePointLv03_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d xyzOffset_ = Eigen::Vector3d::Zero();

  double earthRadiusN_ = 0.0;
  double earthRadiusE_ = 0.0;

  const double equatorialRadius_ = 6378137.0;
  const double flattening_ = 1.0 / 298.257223563;
  const double excentricity2_ = 2 * flattening_ - flattening_ * flattening_;

  const double a_ = 6377397.155;
  const double e2_ = 0.006674372230614;
  const double phi0_ = 46.95241 * M_PI / 180.0;
  const double lambda0_ = 7.43958 * M_PI / 180.0;

  const double r_ = a_ * sqrt(1 - e2_) / (1 - e2_ * sin(phi0_) * sin(phi0_));
  const double alpha_ = sqrt(1 + e2_ / (1 - e2_) * pow(cos(phi0_), 4));
  const double b0_ = asin(sin(phi0_) / alpha_);
  const double k_ = log(tan(M_PI_4 + b0_ / 2.0)) - alpha_ * log(tan(M_PI_4 + phi0_ / 2.0)) +
                    alpha_ * sqrt(e2_) / 2.0 * log((1 + sqrt(e2_) * sin(phi0_)) / (1 - sqrt(e2_) * sin(phi0_)));
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_GNSS_H_