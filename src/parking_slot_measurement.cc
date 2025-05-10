/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/src/parking_slot_measurement.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Thursday, May 8th 2025, 3:17:39 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "parking_slot_measurement.h"

namespace apa_slam {
ParkingSlotMea::ParkingSlotMea(const double timestamp, double* data)
    : SemanticMea(SEMANTIC_TYPE_PARKING_SLOT, timestamp),
      _data(Eigen::Map<Eigen::MatrixXd, Eigen::ColMajor>(
          data, PARKING_SLOT_DATA_ROWS, PARKING_SLOT_DATA_COLS)) {}

Eigen::MatrixXd ParkingSlotMea::GetMeaData() { return _data; }

void ParkingSlotMea::AddNoise(const Eigen::VectorXd& noise) {}
}  // namespace apa_slam
