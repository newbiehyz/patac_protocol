/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/semantic_measurement.h Path:
 * /home/yukan/Documents/work/local_mapping/include Created Date: Thursday, May
 * 8th 2025, 1:34:13 pm Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once

#include <Eigen/Eigen>
#include <memory>

#include "local_mapping_define.h"
#include "apa_parameters.h"
namespace apa_slam {
class SemanticMea {
 public:
  typedef std::shared_ptr<SemanticMea> Ptr;
  SemanticMea(const SensorType& type, const double timestamp);
  virtual ~SemanticMea() = default;
  SensorType GetSemanticMeaType();
  virtual Eigen::MatrixXd GetMeaData() = 0;
  virtual void SetMeaData(const Eigen::MatrixXd &data) = 0;
  virtual void AddNoise() = 0;
  double GetMeaTimestamp();
  virtual Eigen::MatrixXd GetMeasurementNosise() = 0;
  virtual Eigen::VectorXd GetVectorizedData() = 0;
 private:
  SensorType _type;
  double _timestamp;
};
}  // namespace apa_slam
