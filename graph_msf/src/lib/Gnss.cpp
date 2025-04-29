#include "graph_msf/gnss/Gnss.h"

namespace graph_msf {

Gnss::Gnss() {
  calculateConversionParameters();
}

void Gnss::calculateConversionParameters() {
  const double lat_rad = referenceLatitude_ * M_PI / 180.0;
  const double sin_lat = sin(lat_rad);
  const double temp = 1.0 / (1.0 - excentricity2_ * sin_lat * sin_lat);
  const double prime_vertical_radius = equatorialRadius_ * sqrt(temp);
  earthRadiusN_ = prime_vertical_radius * (1.0 - excentricity2_) * temp;
  earthRadiusE_ = prime_vertical_radius * cos(lat_rad);
}

void Gnss::setReference(const double& referenceLatitude, const double& referenceLongitude, const double& referenceAltitude,
                        const double& referenceHeading) {
  referenceLatitude_ = referenceLatitude;
  referenceLongitude_ = referenceLongitude;
  referenceAltitude_ = referenceAltitude;
  referenceHeading_ = referenceHeading;

  calculateConversionParameters();
  referencePointLv03_ = gnssToLv03Raw(referenceLatitude_, referenceLongitude_, referenceAltitude_);
}

void Gnss::setMercatorReferenceFrame(const Eigen::Vector3d& newReferencePoint) {
  xyzOffset_ = newReferencePoint;
}

Eigen::Vector3d Gnss::gnssToCartesian(const double& latitudeInDegrees, const double& longitudeInDegrees, const double& altitude) {
  const double cn = cos(referenceHeading_);
  const double sn = sin(referenceHeading_);
  const double kn = 180.0 / earthRadiusN_ / M_PI;
  const double ke = 180.0 / earthRadiusE_ / M_PI;
  const double dLat = (latitudeInDegrees - referenceLatitude_) / kn;
  const double dLon = (longitudeInDegrees - referenceLongitude_) / ke;

  Eigen::Vector3d position;
  position(0) = cn * dLat + sn * dLon;
  position(1) = sn * dLat - cn * dLon;
  position(2) = altitude - referenceAltitude_;
  return position;
}

Eigen::Vector3d Gnss::cartesianToGps(const Eigen::Ref<const Eigen::Vector3d>& position) const {
  Eigen::Vector3d gpsCoordinates;
  gpsCoordinates(0) =
      referenceLatitude_ + (cos(referenceHeading_) * position(0) + sin(referenceHeading_) * position(1)) / earthRadiusN_ * 180.0 / M_PI;
  gpsCoordinates(1) =
      referenceLongitude_ - (-sin(referenceHeading_) * position(0) + cos(referenceHeading_) * position(1)) / earthRadiusE_ * 180.0 / M_PI;
  gpsCoordinates(2) = referenceAltitude_ + position(2);
  return gpsCoordinates;
}

Eigen::Vector3d Gnss::besselEllipsoidToMercator(const double& latitudeInRad, const double& longitudeInRad, const double& altitude) {
  const double sin_lat = sin(latitudeInRad);
  const double S = alpha_ * log(tan(M_PI_4 + latitudeInRad / 2.0)) -
                   alpha_ * sqrt(e2_) / 2.0 * log((1 + sqrt(e2_) * sin_lat) / (1 - sqrt(e2_) * sin_lat)) + k_;
  const double b = 2.0 * (atan(exp(S)) - M_PI_4);
  const double l = alpha_ * (longitudeInRad - lambda0_);

  const double lHat = atan(sin(l) / (sin(b0_) * tan(b) + cos(b0_) * cos(l)));
  const double bHat = asin(cos(b0_) * sin(b) - sin(b0_) * cos(b) * cos(l));

  const double Y = r_ * lHat;
  const double X = r_ / 2.0 * log((1 + sin(bHat)) / (1 - sin(bHat)));

  return Eigen::Vector3d(Y, X, altitude) - xyzOffset_;
}

Eigen::Vector3d Gnss::gnssToLv03(const double latitudeInDegrees, const double longitudeInDegrees, const double altitude) {
  const Eigen::Vector3d lv03 = gnssToLv03Raw(latitudeInDegrees, longitudeInDegrees, altitude);
  return lv03 - referencePointLv03_;
}

Eigen::Vector3d Gnss::gnssToLv03Raw(const double latitudeInDegrees, const double longitudeInDegrees, const double altitude) {
  const double east = wGStoCHy(latitudeInDegrees, longitudeInDegrees);
  const double north = wGStoCHx(latitudeInDegrees, longitudeInDegrees);
  return Eigen::Vector3d(east, north, altitude);
}

double Gnss::wGStoCHx(double lat, double lon) {
  lat = sexAngleToSeconds(decToSexAngle(lat));
  lon = sexAngleToSeconds(decToSexAngle(lon));
  const double latAux = (lat - 169028.66) / 10000.0;
  const double lonAux = (lon - 26782.5) / 10000.0;

  return 200147.07 + 308807.95 * latAux + 3745.25 * lonAux * lonAux + 76.63 * latAux * latAux - 194.56 * lonAux * lonAux * latAux +
         119.79 * latAux * latAux * latAux;
}

double Gnss::wGStoCHy(double lat, double lon) {
  lat = sexAngleToSeconds(decToSexAngle(lat));
  lon = sexAngleToSeconds(decToSexAngle(lon));
  const double latAux = (lat - 169028.66) / 10000.0;
  const double lonAux = (lon - 26782.5) / 10000.0;

  return 600072.37 + 211455.93 * lonAux - 10938.51 * lonAux * latAux - 0.36 * lonAux * latAux * latAux - 44.54 * lonAux * lonAux * lonAux;
}

double Gnss::decToSexAngle(double dec) {
  const int degree = static_cast<int>(std::floor(dec));
  const int minute = static_cast<int>(std::floor((dec - degree) * 60.0));
  const double second = (((dec - degree) * 60.0) - minute) * 60.0;
  return degree + (static_cast<double>(minute) / 100.0) + (second / 10000.0);
}

double Gnss::sexAngleToSeconds(double dms) {
  const int degree = static_cast<int>(std::floor(dms));
  const int minute = static_cast<int>(std::floor((dms - degree) * 100.0));
  const double second = (((dms - degree) * 100.0) - minute) * 100.0;
  return second + (minute * 60.0) + (degree * 3600.0);
}

}  // namespace graph_msf
