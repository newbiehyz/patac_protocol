/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/odo_measurement.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Thursday, May 8th 2025, 3:41:06 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "odo_measurement.h"

namespace apa_slam {
OdoMea::OdoMea(const double timestamp, double* data)
    : KinematicMea(KINEMATIC_TYPE_ODO, timestamp),
      _data(Eigen::Map<Eigen::MatrixXd, Eigen::ColMajor>(data, DATA_ROWS_ODO,
                                                         1)) {}

Eigen::VectorXd OdoMea::GetMeaData() { return _data; }

void OdoMea::AddNoise() {
  std::default_random_engine generator;
  std::normal_distribution<double> dist(0.0, 1.0);

  Eigen::VectorXd noise = Eigen::VectorXd::Zero(_data.size());
  Eigen::VectorXd stdDev = Eigen::VectorXd::Zero(_data.size());
  stdDev[0] = ApaParameters::GetInstance().GetSimulationParameters().odo_velocity_noise;
  stdDev[1] = ApaParameters::GetInstance().GetSimulationParameters().odo_angular_velocity_noise;

  for (int i = 0; i < noise.size(); ++i) {
    noise(i) = stdDev(i) * dist(generator);  // 直接用标准差缩放
  }

  _data += noise;
}
}  // namespace apa_slam
