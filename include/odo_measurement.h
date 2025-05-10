/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/odo_measurement.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Thursday, May 8th 2025, 2:36:29 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once
#include "kinematic_measurement.h"

namespace apa_slam {
class OdoMea : public KinematicMea {
 public:
  OdoMea(const double timestamp, double* data);
  Eigen::VectorXd GetMeaData();
  void AddNoise(const Eigen::VectorXd& noise);

 private:
  Eigen::VectorXd _data;
};
}  // namespace apa_slam
