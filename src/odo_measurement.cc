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
      _data(Eigen::Map<Eigen::MatrixXd, Eigen::ColMajor>(data, ODO_DATA_ROWS,
                                                         1)) {}

Eigen::VectorXd OdoMea::GetMeaData() { return _data; }
void OdoMea::AddNoise(const Eigen::VectorXd& noise) {}
}  // namespace apa_slam
