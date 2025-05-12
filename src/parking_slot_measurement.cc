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
          data, DATA_ROWS_PARKING_SLOT, DATA_COLS_PARKING_SLOT)) {}

Eigen::MatrixXd ParkingSlotMea::GetMeaData() { return _data; }

void ParkingSlotMea::SetMeaData(const Eigen::MatrixXd& data) { _data = data; };

void ParkingSlotMea::AddNoise() {
  std::default_random_engine generator;
  std::normal_distribution<double> dist(0.0, 1.0);

  Eigen::MatrixXd noise = Eigen::MatrixXd::Zero(_data.rows(), _data.cols());
  Eigen::VectorXd stdDev = Eigen::VectorXd::Zero(_data.cols());
  stdDev[0] =
      ApaParameters::GetInstance().GetSimulationParameters().slot_mea_noise_x;
  stdDev[1] =
      ApaParameters::GetInstance().GetSimulationParameters().slot_mea_noise_y;

  for (int i = 0; i < _data.cols(); ++i) {
    for (int j = 0; j < _data.rows(); ++j) {
      noise(i, j) = stdDev(j) * dist(generator); 
    }
  }

  _data += noise;
}
}  // namespace apa_slam
