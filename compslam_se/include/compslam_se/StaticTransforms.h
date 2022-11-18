#ifndef STATICTRANSFORMS_H
#define STATICTRANSFORMS_H

#include <Eigen/Eigen>
#include <iostream>

namespace compslam_se {

#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

class StaticTransforms {
 public:
  // Constructor
  StaticTransforms() { std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " StaticTransforms instance created." << std::endl; }

  // Setters
  void set_T_frame1_frame2_andInverse(const std::string& frame1, const std::string& frame2, const Eigen::Matrix4d& T_frame1_frame2) {
    lv_T_frame1_frame2(frame1, frame2) = T_frame1_frame2;
    lv_T_frame1_frame2(frame2, frame1) = rv_T_frame1_frame2(frame1, frame2).inverse();
  }

  void setImuFrame(const std::string& s) { imuFrame_ = s; }

  // Getters
  // Returns a left value of the requested transformation
  Eigen::Matrix4d& lv_T_frame1_frame2(const std::string& frame1, const std::string& frame2) {
    std::pair<std::string, std::string> framePair(frame1, frame2);
    return T_frame1_frame2_map_[framePair];
  }

  // Returns a right value to the requested transformation
  const Eigen::Matrix4d& rv_T_frame1_frame2(const std::string& frame1, const std::string& frame2) {
    std::pair<std::string, std::string> framePair(frame1, frame2);
    auto keyIterator = T_frame1_frame2_map_.find(framePair);
    if (keyIterator == T_frame1_frame2_map_.end()) {
      std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " No transform found for " << frame1 << " and " << frame2
                << std::endl;
      throw std::runtime_error("No transform found for " + frame1 + " and " + frame2);
    }
    return keyIterator->second;
  }

  const std::string& getImuFrame() { return imuFrame_; }

  // Functionality
  virtual void findTransformations() = 0;

 protected:
  // General container class
  std::map<std::pair<std::string, std::string>, Eigen::Matrix4d> T_frame1_frame2_map_;

  // Required frames
  std::string imuFrame_;  // If used --> copied to imuCabinFrame_
};

}  // namespace compslam_se

#endif  // STATICTRANSFORMS_H
