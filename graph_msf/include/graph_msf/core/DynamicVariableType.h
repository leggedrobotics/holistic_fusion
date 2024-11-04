//
// Created by nubertj on 04.11.24.
//

#ifndef GRAPH_MSF_DYANAMIC_VARIABLE_TYPE_H
#define GRAPH_MSF_DYANAMIC_VARIABLE_TYPE_H

namespace graph_msf {

enum class DynamicVariableTypeEnum { Global, RefFrame, Landmark };

// Explanation:
// 1) global variables are constant along the entire operation, hence they are not removed and will get the same gtsam key upon return
// 2) reference frames can change, but are modelled according to random walk, hence they are only removed once a successor is initialized
// 3) landmarks are initialized and removed as needed, hence they are removed once they are not needed anymore
// Frames that are not needed but also not removed are set to be "inactive"

// Robust Norm Container
struct DynamicVariableType {
  DynamicVariableType(const DynamicVariableTypeEnum& variableTypeEnum, const Eigen::Vector3d& referenceFrameKeyframePosition,
               const double variableCreationTime)
      : variableTypeEnum_(variableTypeEnum),
        referenceFrameKeyframePosition_(referenceFrameKeyframePosition),
        variableCreationTime_(variableCreationTime) {}

  // Syntactic Sugar for Constructor
  static DynamicVariableType Global(const double variableCreationTime) {
    return DynamicVariableType(DynamicVariableTypeEnum::Global, Eigen::Vector3d::Zero(), variableCreationTime);
  }
  static DynamicVariableType RefFrame(const Eigen::Vector3d& referenceFrameKeyframePosition, const double variableCreationTime) {
    return DynamicVariableType(DynamicVariableTypeEnum::RefFrame, referenceFrameKeyframePosition, variableCreationTime);
  }
  static DynamicVariableType Landmark(const double variableCreationTime) {
    return DynamicVariableType(DynamicVariableTypeEnum::Landmark, Eigen::Vector3d::Zero(), variableCreationTime);
  }

  // Getters
  [[nodiscard]] const DynamicVariableTypeEnum& getVariableTypeEnum() const { return variableTypeEnum_; }
  [[nodiscard]] const Eigen::Vector3d& getReferenceFrameKeyframePosition() const { return referenceFrameKeyframePosition_; }
  [[nodiscard]] double getVariableCreationTime() const { return variableCreationTime_; }

 private:
  // Standard Members
  DynamicVariableTypeEnum variableTypeEnum_;
  Eigen::Vector3d referenceFrameKeyframePosition_;
  double variableCreationTime_;
};

} // namespace graph_msf

#endif  // GRAPH_MSF_DYANAMIC_VARIABLE_TYPE_H
