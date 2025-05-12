/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/parking_slot_measurement.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Thursday, May 8th 2025, 1:46:47 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once

#include <iostream>
#include <memory>
#include <random>
#include <Eigen/Eigen>

#include "semantic_measurement.h"
#include "apa_parameters.h"
namespace apa_slam {
class ParkingSlotMea : public SemanticMea {
 public:
  typedef std::shared_ptr<ParkingSlotMea> Ptr;
  ParkingSlotMea(const double timestamp, double* data);
  Eigen::MatrixXd GetMeaData();
  void AddNoise();
  void SetMeaData(const Eigen::MatrixXd &data);
 private:
  Eigen::MatrixXd _data;
};
}  // namespace apa_slam
