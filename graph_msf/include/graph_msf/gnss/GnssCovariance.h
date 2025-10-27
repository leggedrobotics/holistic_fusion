#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>  // for std::max
#include "graph_msf/gnss/GnssHandler.h"

namespace graph_msf {
namespace gnss_cov {

/** WGS-84 auxiliary radii at latitude lat_rad and height h [m] */
inline void wgs84Radii(double lat_rad, double h, double& M, double& N) {
  constexpr double a  = 6378137.0;                 // semi-major [m]
  constexpr double f  = 1.0 / 298.257223563;       // flattening
  const double e2 = f * (2.0 - f);                 // eccentricity^2
  const double s  = std::sin(lat_rad);
  const double d  = 1.0 - e2 * s * s;
  N = a / std::sqrt(d);
  M = a * (1.0 - e2) / std::pow(d, 1.5);
}

/**
 * Jacobian of (lat[deg], lon[deg], alt[m]) wrt ENU [m] at (lat_deg, h)
 * rows = {lat, lon, alt}, cols = {E, N, U}
 */
inline Eigen::Matrix3d J_lla_wrt_enu(double lat_deg, double h_m) {
  const double lat = lat_deg * M_PI / 180.0;
  double M, N; wgs84Radii(lat, h_m, M, N);
  Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
  J(0,1) = (180.0 / M_PI) / (M + h_m);                      // dlat/dN [deg/m]
  J(1,0) = (180.0 / M_PI) / ((N + h_m) * std::cos(lat));    // dlon/dE [deg/m]
  J(2,2) = 1.0;                                             // dalt/dU
  return J;
}

/**
 * Jacobian of LV03 (E,N,Z) wrt LLA (lat[deg], lon[deg], alt[m])
 * Computed by central differences using your existing converter.
 * NOTE: GnssHandler is intentionally non-const (converter is non-const).
 */
inline Eigen::Matrix3d J_lv03_wrt_lla(graph_msf::GnssHandler& gh,
                                      double lat_deg, double lon_deg, double alt_m) {
  const double ddeg = 1e-6; // ~0.11 m in latitude; small but stable
  const Eigen::Vector3d lla(lat_deg, lon_deg, alt_m);

  Eigen::Vector3d p0; gh.convertNavSatToPositionLV03(lla, p0);

  // d/d(lat)
  Eigen::Vector3d p_p, p_m;
  gh.convertNavSatToPositionLV03(Eigen::Vector3d(lat_deg + ddeg, lon_deg, alt_m), p_p);
  gh.convertNavSatToPositionLV03(Eigen::Vector3d(lat_deg - ddeg, lon_deg, alt_m), p_m);
  const Eigen::Vector2d d_lat = (p_p.head<2>() - p_m.head<2>()) / (2.0 * ddeg);

  // d/d(lon)
  gh.convertNavSatToPositionLV03(Eigen::Vector3d(lat_deg, lon_deg + ddeg, alt_m), p_p);
  gh.convertNavSatToPositionLV03(Eigen::Vector3d(lat_deg, lon_deg - ddeg, alt_m), p_m);
  const Eigen::Vector2d d_lon = (p_p.head<2>() - p_m.head<2>()) / (2.0 * ddeg);

  // Assemble: altitude passes through in your converter
  Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
  J(0,0) = d_lat(0); J(0,1) = d_lon(0);  // dE/dlat(deg), dE/dlon(deg)
  J(1,0) = d_lat(1); J(1,1) = d_lon(1);  // dN/dlat(deg), dN/dlon(deg)
  J(2,2) = 1.0;                           // dZ/dalt(m)
  return J;
}

/** Chain rule: A = d(LV03)/d(ENU) */
inline Eigen::Matrix3d A_lv03_wrt_enu(graph_msf::GnssHandler& gh,
                                      double lat_deg, double lon_deg, double alt_m) {
  const Eigen::Matrix3d J1 = J_lv03_wrt_lla(gh, lat_deg, lon_deg, alt_m); // [m/deg, m/deg, m/m]
  const Eigen::Matrix3d J2 = J_lla_wrt_enu(lat_deg, alt_m);               // [deg/m, deg/m, m/m]
  return J1 * J2; // [m/m]
}

/** Rotate covariance from ENU to LV03 at the given fix. Returns PSD, symmetrized. */
inline Eigen::Matrix3d rotateCov_ENU_to_LV03(graph_msf::GnssHandler& gh,
                                             double lat_deg, double lon_deg, double alt_m,
                                             const Eigen::Matrix3d& P_enu) {
  const Eigen::Matrix3d A = A_lv03_wrt_enu(gh, lat_deg, lon_deg, alt_m);
  Eigen::Matrix3d P = A * P_enu * A.transpose();
  // Hygiene
  P = 0.5 * (P + P.transpose());
  for (int i = 0; i < 3; ++i) P(i,i) = std::max(P(i,i), 1e-12); // clamp to >= 1e-12 m^2
  return P;
}

/** Optional: quick PSD check (debug builds). */
inline bool isPSD(const Eigen::Matrix3d& P) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P);
  if (es.info() != Eigen::Success) return false;
  return es.eigenvalues().minCoeff() >= -1e-10;
}

/** Optional: runtime self-check for A via finite differences */
inline bool selfCheckA(graph_msf::GnssHandler& gh, double lat_deg, double lon_deg, double alt_m) {
  const Eigen::Matrix3d A = A_lv03_wrt_enu(gh, lat_deg, lon_deg, alt_m);

  // Finite-diff columns via small ENU displacements
  const double eps = 0.1; // 10 cm
  auto toLV03 = [&](double dE, double dN, double dU)->Eigen::Vector3d {
    // ENU->LLA (linearized)
    Eigen::Matrix3d J = J_lla_wrt_enu(lat_deg, alt_m);
    Eigen::Vector3d dlla = J * Eigen::Vector3d(dE, dN, dU);
    Eigen::Vector3d p0, p1;
    gh.convertNavSatToPositionLV03(Eigen::Vector3d(lat_deg, lon_deg, alt_m), p0);
    gh.convertNavSatToPositionLV03(Eigen::Vector3d(lat_deg + dlla(0), lon_deg + dlla(1), alt_m + dlla(2)), p1);
    return p1 - p0;
  };
  Eigen::Matrix3d A_fd;
  A_fd.col(0) = toLV03(eps,0,0)/eps;
  A_fd.col(1) = toLV03(0,eps,0)/eps;
  A_fd.col(2) = toLV03(0,0,eps)/eps;

  const double rel = (A - A_fd).norm() / std::max(1.0, A.norm());
  return rel < 1e-4; // typical ~1e-6 to 1e-7
}

} // namespace gnss_cov
} // namespace graph_msf
