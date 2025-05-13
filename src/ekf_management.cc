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
  double noise_v =
      ApaParameters::GetInstance().GetSimulationParameters().odo_velocity_noise;
  double noise_w = ApaParameters::GetInstance()
                       .GetSimulationParameters()
                       .odo_angular_velocity_noise;

  _N = Eigen::MatrixXd::Zero(NOISE_ODO_SIZE, NOISE_ODO_SIZE);
  _N.diagonal()[0] = noise_v * noise_v;
  _N.diagonal()[1] = noise_w * noise_w;

  this->ClearList();
}

EKFManagement &EKFManagement::GetInstance() {
  static EKFManagement instance;
  return instance;
}

void EKFManagement::ClearList() {
  _marginalization_list.clear();
  _update_list.clear();
  _augmentation_list.clear();
}

void EKFManagement::ClearStateList() { _state_list.clear(); }

void EKFManagement::Update(const double timestamp) {
  ekf_update();
  state_augmentation();
  state_marginalization();

  this->ClearList();

  _ts = timestamp;
}

void EKFManagement::ekf_update() {
  if (_update_list.empty()) {
    return;
  }
}

Eigen::MatrixXd EKFManagement::construct_P() {
  int n_state_size = STATE_VEHICLE_SIZE;
  for (auto it = _state_list.begin(); it != _state_list.end(); ++it) {
    switch (it->first) {
      case SEMANTIC_TYPE_PARKING_SLOT:
        n_state_size += STATE_PARKING_SLOT_SIZE;
        break;

      default:
        break;
    }
  }

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n_state_size, n_state_size);
  P.topLeftCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) = _vehicle_cov;

  return P;
}

void EKFManagement::state_augmentation() {
  if (_augmentation_list.empty()) {
    return;
  }
  Eigen::MatrixXd P = this->construct_P();
  for (auto it = _augmentation_list.begin(); it != _augmentation_list.end();
       ++it) {
    for (size_t i = 0; i < it->second.size(); ++i) {
      auto semantic_type = it->first;
      auto landmark_id = it->second.at(i);

      if (!SemanticMap::GetInstance().HasLandmark(semantic_type, landmark_id)) {
        std::cout << "FATAL ERROR, NO SUCH LANDMARK!!!!!!!!!\n";
        continue;
      }
      switch (it->first) {
        case SEMANTIC_TYPE_PARKING_SLOT: {
          Eigen::MatrixXd Jx;
          SemanticMap::GetInstance().InitializeLandmark(semantic_type,
                                                        landmark_id, _vehicle_state, _vehicle_cov, Jx);
          break;
        }

        default:
          break;
      }
    }
  }
}
void EKFManagement::state_marginalization() {}

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
  Eigen::Vector2d dir = Rwb.col(0);

  Eigen::MatrixXd Fx = Eigen::MatrixXd::Identity(3, 3);
  Eigen::Vector2d d_dir_yaw(-std::sin(yaw), std::cos(yaw));
  Fx.topRightCorner(2, 1) = d_dir_yaw * v * dt;

  Eigen::MatrixXd Fn = Eigen::MatrixXd::Zero(3, 2);
  Fn.topLeftCorner(2, 1) = dir * dt;
  Fn(2, 1) = dt;

  P_vehicle1 = Fx * P_vehicle0 * Fx.transpose() + Fn * _N * Fn.transpose();

  twb += dir * v * dt;
  yaw += dt * w;
  x_vehicle1.head(2) = twb;
  x_vehicle1[2] = yaw;

  _vehicle_cov = P_vehicle1;
  _vehicle_state = x_vehicle1;
}

}  // namespace apa_slam
