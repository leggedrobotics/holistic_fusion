/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
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

// Purely virtual interface class for measurements
struct Measurement {
 public:
  Measurement(const std::string& measurementName, const int measurementRate)
      : measurementName_(measurementName), measurementRate_(measurementRate) {}

  // GettersPublic Methods
  const std::string& measurementName() const { return measurementName_; }
  int measurementRate() const { return measurementRate_; }

  // Pure Virtual Class
  virtual const MeasurementTypeEnum& measurementTypeEnum() = 0;

 protected:
  // Standard Members
  std::string measurementName_;
  int measurementRate_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_MEASUREMENT_H
