#ifndef DELTAMEASUREMENT_H
#define DELTAMEASUREMENT_H

#include "compslam_se/measurements/Measurement.h"

namespace compslam_se {

struct BinaryMeasurement : public Measurement {
 public:
  BinaryMeasurement(const std::string& measurementName, const std::string& frameName, const int measurementRate, const double timeKm1,
                    const double timeK)
      : Measurement(measurementName, frameName, measurementRate, timeK), timeKm1_(timeKm1) {}

  virtual MeasurementType measurementType() override { return measurementType_; }

 protected:
  MeasurementType measurementType_ = MeasurementType::Binary;
  double timeKm1_;
};

}  // namespace compslam_se

#endif  // DELTAMEASUREMENT_H
