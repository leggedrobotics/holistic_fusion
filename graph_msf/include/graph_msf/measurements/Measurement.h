/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_MEASUREMENT_H
#define GRAPH_MSF_MEASUREMENT_H

#include <Eigen/Eigen>
#include <string>

namespace graph_msf {

// Enum that contains 2 possible measurement types
enum class MeasurementTypeEnum { Unary, Binary };

// Enum that defines whether robust norm should be used
enum class RobustNormEnum { None, Huber, Cauchy, Tukey };

/**
 * @class RobustNorm
 * @brief Class to represent a robust norm for a measurement.
 *
 * Description: This class is used to represent a robust norm for a measurement.
 */
struct RobustNorm {
  RobustNorm(const RobustNormEnum& robustNormEnum, const double& robustNormConstant)
      : robustNormEnum_(robustNormEnum), robustNormConstant_(robustNormConstant) {}

  // Syntactic Sugar for Constructor
  static RobustNorm None() { return RobustNorm(RobustNormEnum::None, 0.0); }
  static RobustNorm Huber(const double& robustNormConstant) { return RobustNorm(RobustNormEnum::Huber, robustNormConstant); }
  static RobustNorm Cauchy(const double& robustNormConstant) { return RobustNorm(RobustNormEnum::Cauchy, robustNormConstant); }
  static RobustNorm Tukey(const double& robustNormConstant) { return RobustNorm(RobustNormEnum::Tukey, robustNormConstant); }

  // Getters
  [[nodiscard]] const RobustNormEnum& robustNormEnum() const { return robustNormEnum_; }
  [[nodiscard]] const double& robustNormConstant() const { return robustNormConstant_; }

 private:
  // Standard Members
  RobustNormEnum robustNormEnum_;
  double robustNormConstant_;
};

/**
 * @class Measurement
 * @brief Class to represent a measurement in the graph. It is a pure virtual class.
 *
 * Description: This class is used to represent a measurement in the graph.
 */
class Measurement {
 public:
  /**
     * @brief Constructor to create a unary measurement.
     *
     * @param measurementName Name of the measurement.
     * @param measurementRate Rate of the measurement. Mostly used for checking in the core whether all is working correctly.
     * @param sensorFrameName Name of the sensor frame.
     * @param sensorFrameCorrectedName Name of the corrected sensor frame.
     * @param robustNorm Robust norm to be used for the measurement.
     * @param measurementTypeEnum Type of the measurement.
     *
  */
  Measurement(const std::string& measurementName, const int measurementRate, const std::string& sensorFrameName,
              const std::string& sensorFrameCorrectedName, const RobustNorm& robustNorm, const MeasurementTypeEnum& measurementTypeEnum)
      : measurementName_(measurementName),
        measurementRate_(measurementRate),
        sensorFrameName_(sensorFrameName),
        sensorFrameCorrectedName_(sensorFrameCorrectedName),
        robustNorm_(robustNorm),
        measurementTypeEnum_(measurementTypeEnum) {}

  // Destructor
  virtual ~Measurement() = default;

  // Getters
  /// Names
  [[nodiscard]] const std::string& measurementName() const { return measurementName_; }
  [[nodiscard]] const std::string& sensorFrameName() const { return sensorFrameName_; }
  [[nodiscard]] const std::string& sensorFrameCorrectedName() const { return sensorFrameCorrectedName_; }
  /// Rest
  [[nodiscard]] int measurementRate() const { return measurementRate_; }
  [[nodiscard]] const RobustNorm& robustNorm() const { return robustNorm_; }
  [[nodiscard]] const RobustNormEnum& robustNormEnum() const { return robustNorm_.robustNormEnum(); }
  [[nodiscard]] const double& robustNormConstant() const { return robustNorm_.robustNormConstant(); }

  // Setters
  void setMeasurementName(std::string measurementName) { measurementName_ = measurementName; }
  void setSensorFrameName(std::string sensorFrameName) { sensorFrameName_ = sensorFrameName; }

  // Pure Virtual Class
  virtual const MeasurementTypeEnum& measurementTypeEnum() = 0;

 protected:
  // Standard Members
  std::string measurementName_;
  int measurementRate_;
  std::string sensorFrameName_;
  std::string sensorFrameCorrectedName_;

  // Enum
  MeasurementTypeEnum measurementTypeEnum_;
  RobustNorm robustNorm_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_MEASUREMENT_H
