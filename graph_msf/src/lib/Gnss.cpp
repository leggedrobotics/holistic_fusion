/*
Copyright 2017 by Dominic Jud, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include "graph_msf/gnss/Gnss.h"

namespace graph_msf {

Gnss::Gnss() {
  calculateConversionParameters();
}

void Gnss::calculateConversionParameters() {
  // calculate earth radii
  const double temp = 1.0 / (1.0 - excentricity2_ * sin(referenceLatitude_ * M_PI / 180.0) * sin(referenceLatitude_ * M_PI / 180.0));
  const double prime_vertical_radius = equatorialRadius_ * sqrt(temp);
  earthRadiusN_ = prime_vertical_radius * (1 - excentricity2_) * temp;
  earthRadiusE_ = prime_vertical_radius * cos(referenceLatitude_ * M_PI / 180.0);
}

void Gnss::setReference(const double& referenceLatitude, const double& referenceLongitude, const double& referenceAltitude,
                        const double& referenceHeading) {
  referenceLatitude_ = referenceLatitude;
  referenceLongitude_ = referenceLongitude;
  referenceAltitude_ = referenceAltitude;
  referenceHeading_ = referenceHeading;

  calculateConversionParameters();
}

void Gnss::setReferencelv03(const double& referenceLatitude, const double& referenceLongitude, const double& referenceAltitude,
                            const double& referenceHeading) {
  Eigen::Vector3d referenceLv03 = gpsToLv03(referenceLatitude, referenceLongitude, referenceAltitude);

  referenceLatitude_ = referenceLv03[0];
  referenceLongitude_ = referenceLv03[1];
  referenceAltitude_ = referenceLv03[2];
  referenceHeading_ = referenceHeading;

  // calculateConversionParameters();
}

Eigen::Vector3d Gnss::gpsToCartesian(const double& latitudeInDegrees, const double& longitudeInDegrees, const double& altitude) {
  const double cn = cos(referenceHeading_);
  const double sn = sin(referenceHeading_);
  const double kn = 180.0 / earthRadiusN_ / M_PI;
  const double ke = 180.0 / earthRadiusE_ / M_PI;
  const double lat_tmp = (latitudeInDegrees - referenceLatitude_) / kn;
  const double lon_tmp = (longitudeInDegrees - referenceLongitude_) / ke;

  Eigen::Vector3d position;
  position(0) = cn * lat_tmp + sn * lon_tmp;
  position(1) = sn * lat_tmp - cn * lon_tmp;
  position(2) = altitude - referenceAltitude_;

  return position;
}

Eigen::Vector3d Gnss::cartesianToGps(const Eigen::Matrix<double, 3, 1> position) {
  Eigen::Vector3d gpsCoordinates;
  gpsCoordinates(0) =
      referenceLatitude_ + (cos(referenceHeading_) * position(0) + sin(referenceHeading_) * position(1)) / earthRadiusN_ * 180.0 / M_PI;
  gpsCoordinates(1) =
      referenceLongitude_ - (-sin(referenceHeading_) * position(0) + cos(referenceHeading_) * position(1)) / earthRadiusE_ * 180.0 / M_PI;
  gpsCoordinates(2) = referenceAltitude_ + position(2);

  return gpsCoordinates;
}

void Gnss::setMercatorReferenceFrame(const Eigen::Vector3d newReferencePoint) {
  xyzOffset_ = newReferencePoint;
}

Eigen::Vector3d Gnss::besselEllipsoidToMercator(const double& latitudeInRad, const double& longitudeInRad, const double& altitude) {
  // Ellipsoid to Sphere (Gauss)
  const double S = alpha_ * log(tan(M_PI_4 + latitudeInRad / 2.0)) -
                   alpha_ * sqrt(e2_) / 2.0 * log((1 + sqrt(e2_) * sin(latitudeInRad)) / (1 - sqrt(e2_) * sin(latitudeInRad))) + k_;
  const double b = 2 * (atan(exp(S)) - M_PI_4);
  const double l = alpha_ * (longitudeInRad - lambda0_);

  // Equator to Pseudoequator (Rotation)
  const double lHat = atan(sin(l) / (sin(b0_) * tan(b) + cos(b0_) * cos(l)));
  const double bHat = asin(cos(b0_) * sin(b) - sin(b0_) * cos(b) * cos(l));

  // Sphere to Plane (Mercator)
  const double Y = r_ * lHat;
  const double X = r_ / 2.0 * log((1 + sin(bHat)) / (1 - sin(bHat)));

  return Eigen::Vector3d(Y, X, altitude) - xyzOffset_;  // Yes, this is correct. A point in swiss coordinates is denoted as (y,x). See
                                                        // https://en.wikipedia.org/wiki/Swiss_coordinate_system
}

Eigen::Vector3d Gnss::gpsToLv03(const double& latitude_in_degrees, const double& longitude_in_degrees, const double& altitude) {
  Eigen::Vector3d position = gpsToLv03Raw(latitude_in_degrees, longitude_in_degrees, altitude);

  Eigen::Vector3d positionOut = Eigen::Vector3d::Zero();
  positionOut(0) = position(0) - referenceLatitude_;
  positionOut(1) = (position(1) - referenceLongitude_);
  positionOut(2) = position(2) - referenceAltitude_;

  return positionOut;
}

Eigen::Vector3d Gnss::gpsToLv03Raw(const double& latitude_in_degrees, const double& longitude_in_degrees, const double& altitude) {
  double east = WGStoCHy(latitude_in_degrees, longitude_in_degrees);
  double north = WGStoCHx(latitude_in_degrees, longitude_in_degrees);
  return Eigen::Vector3d(east, north, altitude);
}

double Gnss::WGStoCHx(double lat, double lng) {
  lat = SexAngleToSeconds(DecToSexAngle(lat));
  lng = SexAngleToSeconds(DecToSexAngle(lng));

  double lat_aux = (lat - 169028.66) / 10000.0;
  double lng_aux = (lng - 26782.5) / 10000.0;

  double x = (200147.07 + (308807.95 * lat_aux) + (3745.25 * std::pow(lng_aux, 2)) + (76.63 * std::pow(lat_aux, 2)) -
              (194.56 * std::pow(lng_aux, 2) * lat_aux) + (119.79 * std::pow(lat_aux, 3)));
  return x;
}

double Gnss::WGStoCHy(double lat, double lng) {
  lat = SexAngleToSeconds(DecToSexAngle(lat));
  lng = SexAngleToSeconds(DecToSexAngle(lng));

  double lat_aux = (lat - 169028.66) / 10000.0;
  double lng_aux = (lng - 26782.5) / 10000.0;

  double y = (600072.37 + (211455.93 * lng_aux) - (10938.51 * lng_aux * lat_aux) - (0.36 * lng_aux * std::pow(lat_aux, 2)) -
              (44.54 * std::pow(lng_aux, 3)));
  return y;
}

double Gnss::DecToSexAngle(double dec) {
  int degree = std::floor(dec);
  int minute = std::floor((dec - degree) * 60.0);
  double second = (((dec - degree) * 60.0) - minute) * 60.0;
  return degree + (static_cast<double>(minute) / 100.0) + (second / 10000.0);
}

double Gnss::SexAngleToSeconds(double dms) {
  int degree = std::floor(dms);
  int minute = std::floor((dms - degree) * 100.0);
  double second = (((dms - degree) * 100.0) - minute) * 100.0;
  return second + (minute * 60.0) + (degree * 3600.0);
}

}  // namespace graph_msf
