#ifndef YAWFACTOR_H
#define YAWFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Lie.h>

namespace fg_filtering {

/**
 * Factor to estimate rotation given robot heading
 * This version uses model measured bM = scale * bRn * direction
 */
class HeadingFactorYaw : public gtsam::NoiseModelFactor1<gtsam::Pose3> {

public:

  /**
   * Constructor of factor that estimates nav to body rotation bRn
   * @param key of the unknown rotation bRn in the factor graph
   * @param measured magnetometer reading, a 3-vector
   * @param model of the additive Gaussian noise that is assumed
   */
  HeadingFactorYaw(gtsam::Key key, const gtsam::Point3& measuredHeading_, double measuredYaw, const gtsam::SharedNoiseModel& model) :
  gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key), measuredHeading_(measuredHeading_), measuredYaw_(measuredYaw) {
  }

  /// @return a deep copy of this factor
  NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new HeadingFactorYaw(*this)));
  }

  /**
   * @brief vector of errors
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& W_T_I,
                              boost::optional<gtsam::Matrix&> H_Ptr = boost::none) const override {
      // Transform state
      gtsam::Vector1 predictedYaw(W_T_I.rotation().yaw());
      std::cout <<"Predicted yaw: " << predictedYaw << ", measured yaw: " << measuredYaw_ << std::endl;

      // Jacobian
      if (H_Ptr) {
          (*H_Ptr) = (gtsam::Matrix(1,6)<< 0.0, 0.0, 1.0, 0.0, 0.0, 0.0).finished();
      }

      // Compute error
      gtsam::Vector1 difference = predictedYaw - measuredYaw_;
      if (difference(0) > M_PI) {
          difference = difference - gtsam::Vector1(2 * M_PI);
      }
      else if (difference(0) < -M_PI) {
          difference = difference + gtsam::Vector1(2 * M_PI);
      }

      return difference;
  }

private:

  const gtsam::Point3 measuredHeading_;
  const gtsam::Vector1 measuredYaw_;
};

class HeadingFactorMatrix : public gtsam::NoiseModelFactor1<gtsam::Pose3> {

public:

    /**
     * Constructor of factor that estimates nav to body rotation bRn
     * @param key of the unknown rotation bRn in the factor graph
     * @param measured magnetometer reading, a 3-vector
     * @param model of the additive Gaussian noise that is assumed
     */
    HeadingFactorMatrix(gtsam::Key key, const gtsam::Point3& measuredHeading_, double measuredYaw, const gtsam::SharedNoiseModel& model) :
    gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key), measuredHeading_(measuredHeading_), measuredYaw_(measuredYaw) {
    }

    /// @return a deep copy of this factor
    NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new HeadingFactorMatrix(*this)));
    }

    /**
     * @brief vector of errors
     */
      gtsam::Vector evaluateError(const gtsam::Pose3& W_T_I,
                                  boost::optional<gtsam::Matrix&> H_Ptr = boost::none) const override {
        // Transform state
        double predictedYaw = W_T_I.rotation().yaw();
        std::cout <<"Predicted yaw: " << predictedYaw << ", measured yaw: " << measuredYaw_ << std::endl;

        gtsam::Pose3 yawTransformation(gtsam::Rot3::Yaw(predictedYaw), gtsam::Point3(0.0,0.0,0.0));

        // Get rotation and jacobian
        gtsam::Rot3 yawRotation = yawTransformation.rotation(H_Ptr);

        return gtsam::traits<gtsam::Rot3>::Local(yawRotation, gtsam::Rot3::Yaw(measuredYaw_(0)));

        // Rotate x-pointing vector using current state estimate
        //gtsam::Point3 predictedHeading = gtsam::Rot3::Yaw(predictedYaw).rotate(gtsam::Point3(1.0, 0.0, 0.0));
    //    gtsam::Point3 predictedHeading = gtsam::Pose3(gtsam::Rot3::Yaw(predictedYaw), gtsam::Point3(0.0,0.0,0.0)).transformFrom(gtsam::Point3(1.0, 0.0, 0.0), H_Ptr);
    //
    //    if (H_Ptr) {
    //      // assign to temporary first to avoid error in Win-Debug mode
    //      //gtsam::Matrix H_temp = H_Ptr->col(2);
    //      gtsam::Matrix H_temp(3,6);
    //      H_temp << 0.0,0.0,1.0, 0.0,0.0,0.0, // rpy,xyz
    //                0.0,0.0,1.0, 0.0,0.0,0.0, // rpy,xyz
    //                0.0,0.0,1.0, 0.0,0.0,0.0; // rpy,xyz
    //      *H_Ptr = H_temp;
    //    }
    //
    //    return (predictedHeading - measuredHeading_);
      }

private:

    const gtsam::Point3 measuredHeading_;
    const gtsam::Vector1 measuredYaw_;
};

class HeadingFactorHeadingVector : public gtsam::NoiseModelFactor1<gtsam::Pose3> {

public:

    /**
     * Constructor of factor that estimates nav to body rotation bRn
     * @param key of the unknown rotation bRn in the factor graph
     * @param measured magnetometer reading, a 3-vector
     * @param model of the additive Gaussian noise that is assumed
     */
    HeadingFactorHeadingVector(gtsam::Key key, const gtsam::Point3& measuredHeading_, double measuredYaw, const gtsam::SharedNoiseModel& model) :
    gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key), measuredHeading_(measuredHeading_), measuredYaw_(measuredYaw) {
    }

    /// @return a deep copy of this factor
    NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new HeadingFactorHeadingVector(*this)));
    }

    /**
     * @brief vector of errors
     */
    gtsam::Vector evaluateError(const gtsam::Pose3& W_T_I,
                                boost::optional<gtsam::Matrix&> H_Ptr = boost::none) const override {
        // Transform state
        double predictedYaw = W_T_I.rotation().yaw();

        // Rotate x-pointing vector using current state estimate
        //gtsam::Point3 predictedHeading = gtsam::Rot3::Yaw(predictedYaw).rotate(gtsam::Point3(1.0, 0.0, 0.0));
        gtsam::Point3 predictedHeading = gtsam::Pose3(gtsam::Rot3::Yaw(predictedYaw), gtsam::Point3(0.0,0.0,0.0)).transformFrom(gtsam::Point3(1.0, 0.0, 0.0), H_Ptr);

        std::cout << "Jacobian matrix: " << *H_Ptr << std::endl;
        std::cout <<"Predicted heading: " << predictedHeading << ", measured heading: " << measuredHeading_ << std::endl;

        return (predictedHeading - measuredHeading_);
    }

private:

    const gtsam::Point3 measuredHeading_;
    const gtsam::Vector1 measuredYaw_;
};

} // namespace fg_filtering

#endif // YAWFACTOR_H