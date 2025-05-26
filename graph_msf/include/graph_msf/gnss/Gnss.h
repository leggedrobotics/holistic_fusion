#ifndef GRAPH_MSF_GNSS_H_
#define GRAPH_MSF_GNSS_H_

#pragma once

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geoid.hpp>
#include <string>
#include <iostream>
#include <stdexcept>
#include <memory>
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


class SicilianENU {
    public:
     SicilianENU(double lat0Deg,
                 double lon0Deg,
                 double h0Ellip,
                 const std::string& geoidName = "egm2008-1")
       : m_geoidName(geoidName)
     {
       initializeGeoid();
       setAnchor(lat0Deg, lon0Deg, h0Ellip);
     }
   
     struct Anchor {
       double lat0Deg;
       double lon0Deg;
       double h0Ellip;
     };
   
     struct Result {
       double x;        // East  (m)
       double y;        // North (m)
       double zEllip;   // Up    (ellipsoid) (m)
       double zOrth;    // Up    (orthometric) (m)
     };
   
     struct ReverseResult {
       double latDeg;   // Latitude (deg)
       double lonDeg;   // Longitude (deg)
       double hEllip;   // Ellipsoid height (m)
       double hOrth;    // Orthometric height (m)
     };
   
     void setAnchor(double lat0Deg, double lon0Deg, double h0Ellip)
     {
       m_lat0 = lat0Deg;
       m_lon0 = lon0Deg;
       m_h0   = h0Ellip;
       m_local.Reset(lat0Deg, lon0Deg, h0Ellip);
       m_N0      = (*m_geoid)(lat0Deg, lon0Deg);
       m_hOrth0  = h0Ellip - m_N0;
     }
   
     Anchor getAnchor() const
     {
       return {m_lat0, m_lon0, m_h0};
     }
   
     Result forward(double latDeg, double lonDeg, double hEllip) const
     {
       double x, y, z;
       m_local.Forward(latDeg, lonDeg, hEllip, x, y, z);
       double N = (*m_geoid)(latDeg, lonDeg);
       double hOrth = hEllip - N;
       return { x, y, z, hOrth - m_hOrth0 };
     }
   
     ReverseResult backward(double x, double y, double zEllip) const
     {
       double latDeg, lonDeg, hEllip;
       m_local.Reverse(x, y, zEllip, latDeg, lonDeg, hEllip);
       double N = (*m_geoid)(latDeg, lonDeg);
       double hOrth = hEllip - N;
       return { latDeg, lonDeg, hEllip, hOrth };
     }
   
    private:
     void initializeGeoid()
     {
       std::string model = m_geoidName;
       bool geoid_ok = false;
       try {
         m_geoid = std::make_unique<GeographicLib::Geoid>(model);
         geoid_ok = true;
       } catch (const GeographicLib::GeographicErr& e) {
         std::cerr << "[SicilianENU] Could not open geoid \"" << model
                   << "\": " << e.what() << "\n";
         if (model != "egm2008-1") {
           std::cerr << "[SicilianENU] Falling back to \"egm2008-1\".\n";
           try {
             m_geoid = std::make_unique<GeographicLib::Geoid>("egm2008-1");
             model = "egm2008-1";
             geoid_ok = true;
           } catch (const GeographicLib::GeographicErr& e2) {
             std::cerr << "[SicilianENU] Could not open fallback geoid \"egm2008-1\": "
                       << e2.what() << "\n";
           }
         }
       }
       if (!geoid_ok) {
         throw std::runtime_error(
           "[SicilianENU] No geoid model could be loaded. Install \"egm2008-1\" or \"italgeo05\".");
       }
       m_geoidName = model;
       std::cerr << "[SicilianENU] Using geoid: \"" << model << "\"\n";
     }
   
     GeographicLib::LocalCartesian m_local{0,0,0};
     std::unique_ptr<GeographicLib::Geoid> m_geoid;
     std::string m_geoidName;
     double m_lat0 = 0, m_lon0 = 0, m_h0 = 0;
     double m_N0 = 0, m_hOrth0 = 0;
   };
   

}  // namespace graph_msf

#endif  // GRAPH_MSF_GNSS_H_