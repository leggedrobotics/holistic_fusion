#ifndef UNARYMEASUREMENT_H
#define UNARYMEASUREMENT_H

#include "compslam_se/measurements/Measurement.h"

namespace compslam_se {

struct UnaryMeasurement : public Measurement {
 public:
  UnaryMeasurement(const std::string& measurementName, const std::string& frameName, const int measurementRate, const double timeStamp)
      : Measurement(measurementName, frameName, measurementRate, timeStamp) {}

  virtual MeasurementType measurementType() override { return measurementType_; }

 protected:
  MeasurementType measurementType_ = MeasurementType::Unary;
};

}  // namespace compslam_se

#endif  // UNARYMEASUREMENT_H
