/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GTSAMEXPRESSIONTRANSFORMS_H
#define GTSAMEXPRESSIONTRANSFORMS_H

// Output
#define REGULAR_COUT std::cout << YELLOW_START << "GMSF-TransformExpressionKeys" << COLOR_END

// Workspace
#include "graph_msf/core/TransformsDictionary.h"

namespace graph_msf {

class FactorGraphStateKey {
 public:
  // Constructor
  FactorGraphStateKey(const gtsam::Key& key, const double time, const int numberStepsOptimized,
                      const gtsam::Pose3& approximateTransformationBeforeOptimization, const Eigen::Vector3d& measurementOriginPosition)
      : key_(key),
        time_(time),
        numberStepsOptimized_(numberStepsOptimized),
        approximateTransformationBeforeOptimization_(approximateTransformationBeforeOptimization),
        measurementOriginPosition_(measurementOriginPosition) {}
  FactorGraphStateKey() {}

  // Accessors
  const gtsam::Key& key() const { return key_; }
  int getNumberStepsOptimized() const { return numberStepsOptimized_; }
  double getTime() const { return time_; }
  [[nodiscard]] gtsam::Pose3 getApproximateTransformationBeforeOptimization() const { return approximateTransformationBeforeOptimization_; }
  [[nodiscard]] Eigen::Vector3d getMeasurementOriginPosition() const { return measurementOriginPosition_; }

  // Setters
  void setTimeStamp(const double time) { time_ = time; }
  void incrementNumberStepsOptimized() { ++numberStepsOptimized_; }
  void setApproximateTransformationBeforeOptimization(const gtsam::Pose3& approximateTransformationBeforeOptimization) {
    approximateTransformationBeforeOptimization_ = approximateTransformationBeforeOptimization;
  }
  void setMeasurementOriginPosition(const Eigen::Vector3d& measurementOriginPosition) {
    measurementOriginPosition_ = measurementOriginPosition;
  }

 private:
  // Members
  gtsam::Key key_ = -1;
  double time_ = 0.0;
  int numberStepsOptimized_ = 0;
  gtsam::Pose3 approximateTransformationBeforeOptimization_ = gtsam::Pose3();
  Eigen::Vector3d measurementOriginPosition_ = Eigen::Vector3d::Zero();
};

class TransformsExpressionKeys : public TransformsDictionary<FactorGraphStateKey> {
 public:
  // Constructor
  TransformsExpressionKeys() : TransformsDictionary<FactorGraphStateKey>(FactorGraphStateKey()) {
    REGULAR_COUT << " Instance created." << std::endl;
  }

  // Safe Modifiers
  // Returns
  typedef gtsam::Key (*F)(std::uint64_t);
  template <F SYMBOL_SHORTHAND>
  gtsam::Key getTransformationExpression(bool& newGraphKeyAdded, Eigen::Vector3d& modifiedMeasurementOriginPosition,
                                         const std::string& frame1, const std::string& frame2, const double timeK,
                                         const gtsam::Pose3& approximateTransformationBeforeOptimization,
                                         const bool centerMeasurementsAtRobotPositionBeforeAlignment) {
    // Retrieve key and insert information to map
    gtsam::Key T_key;
    std::lock_guard<std::mutex> modifyGraphKeysLock(this->mutex());
    // Case: The dynamically allocated key is not yet in the graph
    newGraphKeyAdded = this->newFramePairSafelyAddedToDictionary<SYMBOL_SHORTHAND>(T_key, modifiedMeasurementOriginPosition, frame1, frame2,
                                                                                   timeK, approximateTransformationBeforeOptimization,
                                                                                   centerMeasurementsAtRobotPositionBeforeAlignment);
    modifiedMeasurementOriginPosition = this->lv_T_frame1_frame2(frame1, frame2).getMeasurementOriginPosition();

    // Return
    return T_key;
  }

  // Overloaded function (without centering)
  template <F SYMBOL_SHORTHAND>
  gtsam::Key getTransformationExpression(bool& newGraphKeyAdded, const std::string& frame1, const std::string& frame2, const double timeK,
                                         const gtsam::Pose3& approximateTransformationBeforeOptimization) {
    Eigen::Vector3d _;  // Placeholder
    getTransformationExpression<SYMBOL_SHORTHAND>(newGraphKeyAdded, _, frame1, frame2, timeK, approximateTransformationBeforeOptimization,
                                                  false);
  }

  template <F SYMBOL_SHORTHAND>
  bool newFramePairSafelyAddedToDictionary(gtsam::Key& returnKey, Eigen::Vector3d& modifiedMeasurementOriginPosition,
                                           const std::string& frame1, const std::string& frame2, const double timeK,
                                           const gtsam::Pose3& approximateTransformationBeforeOptimization,
                                           const bool centerMeasurementsAtRobotPositionBeforeAlignment) {
    // Check and modify content --> acquire lock
    FactorGraphStateKey factorGraphStateKey;
    std::lock_guard<std::mutex> lock(internalDictionaryModifierMutex_);

    // Logic
    // CASE 1: Frame pair is already in dictionary
    if (isFramePairInDictionary(frame1, frame2)) {
      factorGraphStateKey = rv_T_frame1_frame2(frame1, frame2);
      // Update Timestamp if newer
      if (timeK > factorGraphStateKey.getTime()) {
        lv_T_frame1_frame2(frame1, frame2).setTimeStamp(timeK);
        lv_T_frame1_frame2(frame1, frame2).setApproximateTransformationBeforeOptimization(approximateTransformationBeforeOptimization);
      }
      returnKey = factorGraphStateKey.key();
      return false;
    }
    // CASE 2: Frame pair is not in dictionary --> Newly added
    else {
      // If we do not want to move origin --> set to zero
      if (!centerMeasurementsAtRobotPositionBeforeAlignment) {
        modifiedMeasurementOriginPosition = Eigen::Vector3d::Zero();
      }  // else: updatedMeasurementOrigin is set in the function that calls this function
      returnKey = addNewFactorGraphStateKey<SYMBOL_SHORTHAND>(frame1, frame2, timeK, approximateTransformationBeforeOptimization,
                                                              modifiedMeasurementOriginPosition);
      return true;
    }
  }

  // Functionality ------------------------------------------------------------
  template <F SYMBOL_SHORTHAND>
  gtsam::Key addNewFactorGraphStateKey(const std::string& frame1, const std::string& frame2, const double timeK,
                                       const gtsam::Pose3& approximateTransformationBeforeOptimization,
                                       const Eigen::Vector3d& measurementOriginPosition) {
    // Create new key
    gtsam::Key returnKey = SYMBOL_SHORTHAND(getNumberStoredTransformationPairs());
    REGULAR_COUT << GREEN_START << " New key " << gtsam::Symbol(returnKey) << " created for frame pair " << frame1 << " and " << frame2
                 << COLOR_END << std::endl;

    FactorGraphStateKey factorGraphStateKey(returnKey, timeK, 0, approximateTransformationBeforeOptimization, measurementOriginPosition);
    set_T_frame1_frame2(frame1, frame2, factorGraphStateKey);

    // Return
    return returnKey;
  }

  std::mutex& mutex() { return externalModifierMutex_; }

 private:
  // Mutex
  std::mutex internalDictionaryModifierMutex_;
  std::mutex externalModifierMutex_;
};

}  // namespace graph_msf

#endif  // STATIC_TRANSFORMS_H
