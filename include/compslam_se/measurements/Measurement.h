#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <Eigen/Eigen>
#include <string>

namespace compslam_se {

struct Measurement {
 public:
  Measurement() {}

  virtual std::string type() = 0;

  std::string name;
  int rate;

  double time;

  Eigen::MatrixXd measurementNoise;
};

}  // namespace compslam_se

#endif  // MEASUREMENT_H
