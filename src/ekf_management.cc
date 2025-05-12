/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/ekf_management.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Monday, May 12th 2025, 9:19:23 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include "ekf_management.h"

namespace apa_slam {
EKFManagement::EKFManagement() {}

void EKFManagement::Init() {
  // _N = Eigen::MatrixXd::Zero
}

EKFManagement &EKFManagement::GetInstance() {
  static EKFManagement instance;
  return instance;
}

CrossCorrelationKey EKFManagement::make_lm_cross_correlation_key(
    const SensorType &type0, const int &id0, const SensorType &type1,
    const int &id1) {
  auto id_a = CrossCorrelationId{type0, id0};
  auto id_b = CrossCorrelationId{type1, id1};
  return CrossCorrelationKey{id_a, id_b};
}

void EKFManagement::Propagate(const double v, const double w, const double dt,
                              const Eigen::VectorXd &x_vehicle0,
                              const Eigen::MatrixXd &P_vehicle0,
                              Eigen::VectorXd &x_vehicle1,
                              Eigen::MatrixXd &P_vehicle1) {
  x_vehicle1 = x_vehicle0;
  P_vehicle1 = P_vehicle0;

  Eigen::Vector2d twb = x_vehicle1.head(2);
  double yaw = x_vehicle1[2];
  Eigen::Rotation2Dd rot(yaw);
  Eigen::Matrix2d Rwb = rot.toRotationMatrix();
  // Eigen::VEcto
}

}  // namespace apa_slam
