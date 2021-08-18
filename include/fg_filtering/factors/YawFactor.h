#ifndef YAWFACTOR_H
#define YAWFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>

namespace fg_filtering {

/**
 * Factor to estimate rotation given robot heading
 * This version uses model measured bM = scale * bRn * direction
 */
class YawFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {

public:

  /**
   * Constructor of factor that estimates nav to body rotation bRn
   * @param key of the unknown rotation bRn in the factor graph
   * @param measured magnetometer reading, a 3-vector
   * @param model of the additive Gaussian noise that is assumed
   */
  YawFactor(gtsam::Key key, const gtsam::Point3& measuredHeading_, const gtsam::SharedNoiseModel& model) :
  gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key), measuredHeading_(measuredHeading_) {
  }

  /// @return a deep copy of this factor
  NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new YawFactor(*this)));
  }

  /**
   * @brief vector of errors
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& W_T_I,
                              boost::optional<gtsam::Matrix&> H_Ptr = boost::none) const override {
    // Transform state
    double yaw = W_T_I.rotation().yaw();
    // Rotate x-pointing vector using current state estimate
    gtsam::Point3 predictedHeading = gtsam::Rot3::Yaw(yaw).rotate(gtsam::Point3(1.0, 0.0, 0.0));

//    if (H_Ptr) {
//      // assign to temporary first to avoid error in Win-Debug mode
//      //gtsam::Matrix H_temp = H_Ptr->col(2);
//      gtsam::Matrix H_temp(3,6) << 0.0,0.0,0.0,1.0,0.0,0.0, \
//                                         0.0,0.0,0.0,1.0,0.0,0.0, \
//                                         0.0,0.0,0.0,1.0,0.0,0.0;
//      *H_Ptr = H_temp;
//    }

    return (predictedHeading - measuredHeading_);
  }

private:

  const gtsam::Point3 measuredHeading_;
};

} // namespace fg_filtering

#endif // YAWFACTOR_H