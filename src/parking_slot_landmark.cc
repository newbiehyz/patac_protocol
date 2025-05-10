/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/src/parking_slot_landmark.cc Path:
 * /home/yukan/Documents/work/local_mapping/src Created Date: Thursday, May 8th
 * 2025, 6:58:35 pm Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include "parking_slot_landmark.h"

namespace apa_slam {
ParkingSlotLandmark::ParkingSlotLandmark(const int id, double *data)
    : SemanticLandmark(SEMANTIC_TYPE_PARKING_SLOT, id),
      _data(Eigen::Map<Eigen::MatrixXd, Eigen::ColMajor>(
          data, PARKING_SLOT_DATA_ROWS, PARKING_SLOT_DATA_COLS)) {}

Eigen::MatrixXd ParkingSlotLandmark::GetLandmarkData() { return _data; }
}  // namespace apa_slam
