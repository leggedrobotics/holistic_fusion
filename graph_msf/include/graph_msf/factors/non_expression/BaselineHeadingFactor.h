#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>
#include <stdexcept>

namespace graph_msf {

struct BodyBaselineHeading {
  double heading;
  gtsam::Vector3 baselineImu;
};

// Dual-antenna heading is the azimuth of the actual baseline. Treating it as
// CABIN yaw plus a constant offset loses the roll/pitch geometry on slopes.
class BaselineHeadingFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;
  using Base::evaluateError;

  BaselineHeadingFactor(gtsam::Key key, const BodyBaselineHeading& measurement,
                        const gtsam::SharedNoiseModel& noise)
      : Base(noise, key), heading_(measurement.heading), baseline_(measurement.baselineImu) {
    if (!std::isfinite(heading_) || !baseline_.allFinite() || baseline_.norm() < 1e-9) {
      throw std::invalid_argument("Invalid GNSS baseline heading measurement");
    }
    baseline_.normalize();
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new BaselineHeadingFactor(*this));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              gtsam::OptionalMatrixType H = OptionalNone) const override {
    gtsam::Matrix33 H_direction;
    const auto direction = pose.rotation().rotate(baseline_, H_direction);
    const double horizontal_squared = direction.head<2>().squaredNorm();
    if (horizontal_squared < 1e-12) {
      throw std::domain_error("GNSS baseline has undefined azimuth");
    }
    if (H) {
      *H = gtsam::Matrix16::Zero();
      const gtsam::Matrix13 H_azimuth =
          (gtsam::Matrix13() << -direction.y() / horizontal_squared,
           direction.x() / horizontal_squared, 0.0).finished();
      H->leftCols<3>() = H_azimuth * H_direction;
    }
    const double residual = std::atan2(direction.y(), direction.x()) - heading_;
    return gtsam::Vector1(std::atan2(std::sin(residual), std::cos(residual)));
  }

 private:
  double heading_;
  gtsam::Vector3 baseline_;
};

}  // namespace graph_msf
