/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef FACTOR_GRAPH_STATE_KEY_H
#define FACTOR_GRAPH_STATE_KEY_H

// Output
#define REGULAR_COUT std::cout << YELLOW_START << "GMSF-FactorGraphStateKey" << COLOR_END

namespace graph_msf {

enum class VariableTypeEnum { Global, RefFrame, Landmark };

// Explanation:
// 1) global variables are constant along the entire operation, hence they are not removed and will get the same gtsam key upon return
// 2) reference frames can change, but are modelled according to random walk, hence they are only removed once a successor is initialized
// 3) landmarks are initialized and removed as needed, hence they are removed once they are not needed anymore
// Frames that are not needed but also not removed are set to be "inactive"

// Robust Norm Container
struct VariableType {
  VariableType(const VariableTypeEnum& variableTypeEnum, const Eigen::Vector3d& measurementKeyframePosition)
      : variableTypeEnum_(variableTypeEnum), measurementKeyframePosition_(measurementKeyframePosition) {}

  // Syntactic Sugar for Constructor
  static VariableType Global() { return VariableType(VariableTypeEnum::Global, Eigen::Vector3d::Zero()); }
  static VariableType RefFrame(const Eigen::Vector3d& measurementKeyframePosition) {
    return VariableType(VariableTypeEnum::RefFrame, measurementKeyframePosition);
  }
  static VariableType Landmark() { return VariableType(VariableTypeEnum::Landmark, Eigen::Vector3d::Zero()); }

  // Getters
  [[nodiscard]] const VariableTypeEnum& variableTypeEnum() const { return variableTypeEnum_; }
  [[nodiscard]] const Eigen::Vector3d& measurementKeyframePosition() const { return measurementKeyframePosition_; }

 private:
  // Standard Members
  VariableTypeEnum variableTypeEnum_;
  Eigen::Vector3d measurementKeyframePosition_;
};

template <class GTSAM_TRANSFORM_TYPE>  // e.g. gtsam::Pose3, gtsam::Point3
class FactorGraphStateKey {
 public:
  // Constructor
  FactorGraphStateKey(const gtsam::Key& key, const double time, const int numberStepsOptimized,
                      const GTSAM_TRANSFORM_TYPE& approximateTransformationBeforeOptimization, VariableType variableType)
      : key_(key),
        time_(time),
        numberStepsOptimized_(numberStepsOptimized),
        approximateTransformationBeforeOptimization_(approximateTransformationBeforeOptimization),
        variableType_(std::move(variableType)),
        isVariableActive_(true) {}

  // Default constructor for creating identity object
  FactorGraphStateKey() = default;

  // Destructor
  ~FactorGraphStateKey() = default;

  // Accessors
  const gtsam::Key key() const { return key_; }
  int getNumberStepsOptimized() const { return numberStepsOptimized_; }
  double getTime() const { return time_; }
  [[nodiscard]] GTSAM_TRANSFORM_TYPE getApproximateTransformationBeforeOptimization() const {
    return approximateTransformationBeforeOptimization_;
  }
  [[nodiscard]] Eigen::Vector3d getMeasurementKeyframePosition() const { return variableType_.measurementKeyframePosition(); }
  [[nodiscard]] const VariableTypeEnum& getVariableTypeEnum() const { return variableType_.variableTypeEnum(); }
  [[nodiscard]] bool isVariableActive() const { return isVariableActive_; }

  // Setters
  void setTimeStamp(const double time) { time_ = time; }
  void incrementNumberStepsOptimized() { ++numberStepsOptimized_; }
  void setApproximateTransformationBeforeOptimization(const gtsam::Pose3& approximateTransformationBeforeOptimization) {
    approximateTransformationBeforeOptimization_ = approximateTransformationBeforeOptimization;
  }
  void activateVariable() {
    isVariableActive_ = true;
    REGULAR_COUT << " Activated Variable at Key " << gtsam::Symbol(key_) << std::endl;
  }
  void deactivateVariable() {
    isVariableActive_ = false;
    REGULAR_COUT << " Deactivated Variable at Key " << gtsam::Symbol(key_) << std::endl;
  }

 private:
  // Members
  gtsam::Key key_ = -1;
  double time_ = 0.0;
  int numberStepsOptimized_ = 0;
  GTSAM_TRANSFORM_TYPE approximateTransformationBeforeOptimization_;
  VariableType variableType_ = VariableType::Global();
  bool isVariableActive_ = true;  // Always active in the beginning when added
};

}  // namespace graph_msf

#endif  // FACTOR_GRAPH_STATE_KEY_H
