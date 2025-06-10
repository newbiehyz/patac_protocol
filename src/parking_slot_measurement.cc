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
ParkingSlotMea::ParkingSlotMea(const long long timestamp, double* data)
    : SemanticMea(SEMANTIC_TYPE_PARKING_SLOT, timestamp),
      _data(Eigen::Map<Eigen::MatrixXd, Eigen::ColMajor>(
          data, DATA_ROWS_PARKING_SLOT, DATA_COLS_PARKING_SLOT)) {
  double noise_ps_x =
      ApaParameters::GetInstance().GetEstimatorParamters().slot_mea_noise_x;
  double noise_ps_y =
      ApaParameters::GetInstance().GetEstimatorParamters().slot_mea_noise_y;
  _R = Eigen::MatrixXd::Zero(NOISE_PARKING_SLOT, NOISE_PARKING_SLOT);
  _R.diagonal()[0] = noise_ps_x * noise_ps_x;
  _R.diagonal()[1] = noise_ps_y * noise_ps_y;
  _R.diagonal()[2] = noise_ps_x * noise_ps_x;
  _R.diagonal()[3] = noise_ps_y * noise_ps_y;
}

Eigen::MatrixXd ParkingSlotMea::GetMeasurementNosise() { return _R; }

Eigen::VectorXd ParkingSlotMea::GetVectorizedData() {
  Eigen::VectorXd data = Eigen::VectorXd::Zero(STATE_PARKING_SLOT_SIZE);
  data.head(2) = this->_data.col(0).head(2);
  data.tail(2) = this->_data.col(1).head(2);

  return data;
}

Eigen::MatrixXd ParkingSlotMea::GetMeaData() { return _data; }

void ParkingSlotMea::SetMeaData(const Eigen::MatrixXd& data) { _data = data; };

void ParkingSlotMea::SetAttribute(const ParkingSlotAttribute& attri) {
  _attri = attri;
}

void ParkingSlotMea::AddNoise() {
  std::default_random_engine generator;
  std::normal_distribution<double> dist(0.0, 1.0);

  Eigen::MatrixXd noise = Eigen::MatrixXd::Zero(_data.rows(), _data.cols());
  Eigen::VectorXd stdDev = Eigen::VectorXd::Zero(_data.cols());
  stdDev[0] =
      ApaParameters::GetInstance().GetSimulationParameters().slot_mea_noise_x;
  stdDev[1] =
      ApaParameters::GetInstance().GetSimulationParameters().slot_mea_noise_y;

  for (int i = 0; i < _data.rows(); ++i) {
    for (int j = 0; j < _data.cols(); ++j) {
      noise(i, j) = stdDev(j) * dist(generator);
    }
  }
  // std::cout << "Noise Ps\n" << noise << std::endl;
  _data += noise;
}
}  // namespace apa_slam
