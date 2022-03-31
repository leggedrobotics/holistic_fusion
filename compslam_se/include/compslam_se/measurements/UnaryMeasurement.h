#ifndef UNARYMEASUREMENT_H
#define UNARYMEASUREMENT_H

#include "compslam_se/measurements/Measurement.h"

namespace compslam_se {

struct UnaryMeasurement : public Measurement {
 public:
  UnaryMeasurement() {}

  virtual std::string type() override { return name; }

  Eigen::MatrixXd T_k;
};

}  // namespace compslam_se

#endif  // UNARYMEASUREMENT_H
