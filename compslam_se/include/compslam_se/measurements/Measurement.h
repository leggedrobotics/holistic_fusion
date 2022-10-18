#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <Eigen/Eigen>
#include <string>

namespace compslam_se {

// Enum that contains 2 possible measurement types
enum class MeasurementType { Unary, Binary };

struct Measurement {
 public:
  Measurement(const std::string& measurementName, const std::string& frameName, const int measurementRate, const double timeStamp)
      : measurementName_(measurementName), frameName_(frameName), measurementRate_(measurementRate), timeK_(timeStamp) {}

  // Public Methods
  const std::string& measurementName() const { return measurementName_; }
  const std::string& frameName() const { return frameName_; }
  int measurementRate() const { return measurementRate_; }
  double timeK() const { return timeK_; }

  // Pure Virtual Class
  virtual MeasurementType measurementType() = 0;

 protected:
  // Members
  std::string measurementName_;
  std::string frameName_;
  int measurementRate_;
  double timeK_;

  // Unknown at this level
  MeasurementType measurementType_;
};

}  // namespace compslam_se

#endif  // MEASUREMENT_H
