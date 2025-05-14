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

void EKFManagement::ClearList() { _lm_state_list.clear(); }

void EKFManagement::Update(const Eigen::VectorXd &state_mean,
                           const Eigen::MatrixXd &state_P,
                           const double timestamp) {
  _vehicle_state = state_mean;
  _vehicle_cov = state_P;
  std::map<SensorType, std::set<int>> augmentation_list;
  std::map<SensorType, std::set<int>> update_list;
  std::map<SensorType, std::set<int>> marginalization_list;
  SemanticMap::GetInstance().GetEKFDataList(augmentation_list, update_list,
                                            marginalization_list);

  if (!_initialized) {
    return;
  }

  ekf_update(update_list);
  state_augmentation(augmentation_list);
  state_marginalization(marginalization_list);

  _ts = timestamp;
}

void EKFManagement::ekf_update(
    const std::map<SensorType, std::set<int>> &update_list) {
      // for (auto )
    }

int EKFManagement::get_state_size() {
  int n_state_size = STATE_VEHICLE_SIZE;
  for (auto it = _lm_state_list.begin(); it != _lm_state_list.end(); ++it) {
    switch (it->first) {
      case SEMANTIC_TYPE_PARKING_SLOT:
        n_state_size += STATE_PARKING_SLOT_SIZE;
        break;

      default:
        break;
    }
  }

  return n_state_size;
}

Eigen::MatrixXd EKFManagement::construct_P() {
  int n_state_size = get_state_size();

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n_state_size, n_state_size);
  P.topLeftCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) = _vehicle_cov;

  int pos0 = STATE_VEHICLE_SIZE;
  for (auto it_type0 = _lm_state_list.begin(); it_type0 != _lm_state_list.end();
       ++it_type0) {
    const auto &type0 = it_type0->first;
    const auto &set0 = it_type0->second;

    for (auto it_i = set0.begin(); it_i != set0.end(); ++it_i) {
      const int &id0 = *it_i;
      auto it_type1 = it_type0;

      // initialize auto-correlation
      Eigen::MatrixXd auto_correlation =
          SemanticMap::GetInstance().GetLandmarkCov(type0, id0);
      int size0;
      switch (type0) {
        case SEMANTIC_TYPE_PARKING_SLOT:
          size0 = STATE_PARKING_SLOT_SIZE;
          break;

        default:
          break;
      }
      P.block(pos0, pos0, size0, size0) = auto_correlation;

      // cross_correlation with vehicle
      CrossCorrelationId correlation_id0(type0, id0);

      P.block(0, pos0, STATE_VEHICLE_SIZE, size0) =
          _state_lm_cross_correlation.at(correlation_id0);
      P.block(pos0, 0, size0, STATE_VEHICLE_SIZE) =
          P.block(0, 0, STATE_VEHICLE_SIZE, size0).transpose();

      int pos1 = pos0;
      for (; it_type1 != _lm_state_list.end(); ++it_type1) {
        const auto &type1 = it_type1->first;
        const auto &set1 = it_type1->second;

        int size1;
        switch (type1) {
          case SEMANTIC_TYPE_PARKING_SLOT:
            size1 = STATE_PARKING_SLOT_SIZE;
            break;

          default:
            break;
        }
        pos1 += size1;
        auto j_start = (it_type0 == it_type1) ? it_i : set1.begin();
        for (auto it_j = j_start; it_j != set1.end(); ++it_j) {
          const int &id1 = *it_j;
          CrossCorrelationId correlation_id1(type1, id1);

          if (it_type0 != it_type1 || it_i != it_j) {
            // initialize cross-correlation
            P.block(pos0, pos1, size0, size1) =
                get_cross_correlation(correlation_id0, correlation_id1);
            P.block(pos1, pos0, size1, size0) =
                P.block(pos0, pos1, size0, size1).transpose();
          }
        }
      }

      pos0 += size0;
    }
  }

  return P;
}

void EKFManagement::aug_update_covariance(const SensorType &type, const int &id,
                                          const Eigen::MatrixXd &Jx) {
  Eigen::MatrixXd P = construct_P();
  int size;
  switch (type) {
    case SEMANTIC_TYPE_PARKING_SLOT:
      size = STATE_PARKING_SLOT_SIZE;
      break;

    default:
      break;
  }
  int rows = P.rows();
  int cols = P.cols();
  P.conservativeResize(rows + size, cols + size);

  P.topLeftCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) =
      Jx * P.topLeftCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) *
      Jx.transpose();

  P.topRightCorner(STATE_VEHICLE_SIZE, size) =
      P.topLeftCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) * Jx.transpose();
  P.bottomLeftCorner(size, STATE_VEHICLE_SIZE) =
      P.topRightCorner(STATE_VEHICLE_SIZE, size).transpose();

  P.block(STATE_VEHICLE_SIZE, cols, cols - STATE_VEHICLE_SIZE, size) =
      P.block(0, STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE,
              cols - STATE_VEHICLE_SIZE)
          .transpose() *
      Jx.transpose();

  CrossCorrelationId correlation_id(type, id);

  int pos = STATE_VEHICLE_SIZE;
  int correlation_cols = size;

  for (auto it_type = _lm_state_list.begin(); it_type != _lm_state_list.end();
       ++it_type) {
    for (auto it_id = it_type->second.begin(); it_id != it_type->second.end();
         ++it_id) {
      const auto &type0 = it_type->first;
      const auto &lm_id0 = *it_id;
      CrossCorrelationId correlation_id0(type0, lm_id0);
      int correlation_rows;
      switch (type0) {
        case SEMANTIC_TYPE_PARKING_SLOT:
          correlation_rows = STATE_PARKING_SLOT_SIZE;
          break;

        default:
          break;
      }
      Eigen::MatrixXd correlation = P.block(pos, cols, correlation_rows, size);
      CrossCorrelationKey key =
          make_lm_cross_correlation_key(correlation_id, correlation_id);

      set_cross_correlation(correlation_id0, correlation_id, correlation);

      pos += correlation_rows;
    }
  }
}

void EKFManagement::state_augmentation(
    const std::map<SensorType, std::set<int>> &augmentation_list) {
  if (augmentation_list.empty()) {
    return;
  }
  std::cout << "Augmentation List:\n";
  for (auto it = augmentation_list.begin()->second.begin();
       it != augmentation_list.begin()->second.end(); ++it) {
    std::cout << *it << " ";
  }
  std::cout << std::endl;

  for (auto it = augmentation_list.begin(); it != augmentation_list.end();
       ++it) {
    for (auto itt = it->second.begin(); itt != it->second.end(); ++itt) {
      auto semantic_type = it->first;
      auto landmark_id = *itt;

      if (!SemanticMap::GetInstance().HasLandmark(semantic_type, landmark_id)) {
        std::cout << "FATAL ERROR, NO SUCH LANDMARK!!!!!!!!!\n";
        continue;
      }
      switch (semantic_type) {
        case SEMANTIC_TYPE_PARKING_SLOT: {
          Eigen::MatrixXd Jx;
          SemanticMap::GetInstance().InitializeLandmark(
              semantic_type, landmark_id, _vehicle_state, _vehicle_cov, Jx);
          aug_update_covariance(semantic_type, landmark_id, Jx);
          _lm_state_list[semantic_type].insert(landmark_id);
          break;
        }

        default:
          break;
      }
    }
  }
}
void EKFManagement::state_marginalization(
    const std::map<SensorType, std::set<int>> &marginalization_list) {}

CrossCorrelationKey EKFManagement::make_lm_cross_correlation_key(
    const CrossCorrelationId &id0, const CrossCorrelationId &id1) {
  return CrossCorrelationKey(id0, id1);
}

void EKFManagement::set_cross_correlation(const CrossCorrelationId &id0,
                                          const CrossCorrelationId &id1,
                                          const Eigen::MatrixXd &correlation) {
  CrossCorrelationKey key = make_lm_cross_correlation_key(id0, id1);
  if (id0.type == key.first.type && id0.id == key.first.id) {
    _lm_cross_correlation[key] = correlation;
  } else {
    _lm_cross_correlation[key] = correlation.transpose();
  }
}

Eigen::MatrixXd EKFManagement::get_cross_correlation(
    const CrossCorrelationId &id0, const CrossCorrelationId &id1) {
  CrossCorrelationKey key = make_lm_cross_correlation_key(id0, id1);
  if (id0.type == key.first.type && id0.id == key.first.id) {
    return _lm_cross_correlation.at(key);
  } else {
    return _lm_cross_correlation.at(key).transpose();
  }
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

  if (!_initialized) {
    _initialized = true;
  }
}

}  // namespace apa_slam
