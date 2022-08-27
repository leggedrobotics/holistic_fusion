#ifndef COMPSLAM_SE_HEADING_FACTOR_H
#define COMPSLAM_SE_HEADING_FACTOR_H

#include <string>

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/none.hpp>
#include <boost/shared_ptr.hpp>

namespace compslam_se {

/**
 * Factor to estimate rotation given gnss robot heading
 */
class HeadingFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  /**
   * Constructor of factor that estimates nav to body rotation bRn
   * @param key of the unknown rotation bRn in the factor graph
   * @param measured magnetometer reading, a 3-vector
   * @param model of the additive Gaussian noise that is assumed
   */
  HeadingFactor(gtsam::Key j, double yaw, const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, j), yaw_(yaw) {}

  // Destructor
  virtual ~HeadingFactor() {}

  /**
   * Evaluate error function
   * @brief vector of errors
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& q, boost::optional<gtsam::Matrix&> H_Ptr = boost::none) const {
    // Jacobian
    if (H_Ptr) {
      (*H_Ptr) = (gtsam::Matrix(1, 6) << 0.0, 0.0, -1.0, 0.0, 0.0, 0.0).finished();  // [rad] [m]
    }

    // calculate error
    double yaw_error = q.rotation().yaw() - yaw_;

    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;

    return gtsam::Vector1(yaw_error);
  }

 private:
  double yaw_;  // yaw measurement
};

}  // namespace compslam_se

#endif  // COMPSLAM_SE_HEADING_FACTOR_H