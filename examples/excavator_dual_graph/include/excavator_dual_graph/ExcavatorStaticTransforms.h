#ifndef ExcavatorStaticTransforms_H
#define ExcavatorStaticTransforms_H
// Workspace
#include "compslam_se_ros/extrinsics/StaticTransformsTf.h"

namespace excavator_se {

class ExcavatorStaticTransforms : public compslam_se::StaticTransformsTf {
 public:
  ExcavatorStaticTransforms(std::shared_ptr<ros::NodeHandle> privateNodePtr);

  // Setters
  void setMapFrame(const std::string& s) { mapFrame_ = s; }
  void setOdomFrame(const std::string& s) { odomFrame_ = s; }
  void setLidarFrame(const std::string& s) { lidarFrame_ = s; }
  void setLeftGnssFrame(const std::string& s) { leftGnssFrame_ = s; }
  void setRightGnssFrame(const std::string& s) { rightGnssFrame_ = s; }
  void setCabinFrame(const std::string& s) { cabinFrame_ = s; }
  void setBaseLinkFrame(const std::string& s) { baseLinkFrame_ = s; }

  // Getters
  const std::string& getMapFrame() { return mapFrame_; }
  const std::string& getOdomFrame() { return odomFrame_; }
  const std::string& getLidarFrame() { return lidarFrame_; }
  const std::string& getLeftGnssFrame() { return leftGnssFrame_; }
  const std::string& getRightGnssFrame() { return rightGnssFrame_; }
  const std::string& getCabinFrame() { return cabinFrame_; }
  const std::string& getBaseLinkFrame() { return baseLinkFrame_; }

 private:
  void findTransformations() override;

  // Members
  std::string mapFrame_;
  std::string odomFrame_;
  std::string lidarFrame_;
  std::string leftGnssFrame_;
  std::string rightGnssFrame_;
  std::string cabinFrame_;
  std::string baseLinkFrame_;
};
}  // namespace excavator_se
#endif  // end AsopStaticTransforms_H
