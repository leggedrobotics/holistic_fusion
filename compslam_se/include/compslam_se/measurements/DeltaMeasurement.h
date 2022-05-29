#ifndef DELTAMEASUREMENT_H
#define DELTAMEASUREMENT_H

#include "compslam_se/measurements/Measurement.h"

namespace compslam_se {

struct DeltaMeasurement : public Measurement {
 public:
  DeltaMeasurement() {}

  virtual std::string type() override { return name; }

  Eigen::MatrixXd T_delta;
  double timeKm1;
};

}  // namespace compslam_se

#endif  // DELTAMEASUREMENT_H
