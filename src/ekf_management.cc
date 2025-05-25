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
      ApaParameters::GetInstance().GetEstimatorParamters().odo_velocity_noise;
  double noise_w = ApaParameters::GetInstance()
                       .GetEstimatorParamters()
                       .odo_angular_velocity_noise;

  _N = Eigen::MatrixXd::Zero(NOISE_ODO_SIZE, NOISE_ODO_SIZE);
  _N.diagonal()[0] = noise_v * noise_v;
  _N.diagonal()[1] = noise_w * noise_w;

  _vehicle_x = Eigen::VectorXd::Zero(3);
  _vehicle_P = Eigen::MatrixXd::Identity(3, 3) * 0.001;

  _initialized = false;
  this->ClearList();
}

EKFManagement &EKFManagement::GetInstance() {
  static EKFManagement instance;
  return instance;
}

bool EKFManagement::GetLatestVechileState(double &timestamp,
                                          Eigen::VectorXd &mean,
                                          Eigen::MatrixXd &cov) {
  if (!_initialized) {
    return false;
  }
  timestamp = _ts;
  mean = _vehicle_x;
  cov = _vehicle_P;

  return true;
}

bool EKFManagement::Initialized() { return _initialized; }

void EKFManagement::ClearList() { _lm_state_list.clear(); }

void EKFManagement::Update(const double timestamp) {
  std::cout << "Update At: " << timestamp << std::endl;
  
  std::map<SensorType, std::set<int>> augmentation_list;
  std::map<SensorType, std::set<int>> update_list;
  std::map<SensorType, std::set<int>> marginalization_list;
  SemanticMap::GetInstance().GetEKFDataList(augmentation_list, update_list,
                                            marginalization_list);

  if (!_initialized) {
    return;
  }

  state_marginalization(marginalization_list);
  if (!marginalization_list.empty()) {
    for (auto it_type = marginalization_list.begin();
         it_type != marginalization_list.end(); ++it_type) {
      const auto type = it_type->first;
      for (auto it = it_type->second.begin(); it != it_type->second.end();
           ++it) {
        int id = *it;
        if (update_list.count(type) && update_list.at(type).count(id)) {
          update_list.at(type).erase(id);
        }
      }
    }
  }

  ekf_update(update_list);
  state_augmentation(augmentation_list);
  SemanticMap::GetInstance().TagMarginalization(timestamp);
  // _ts = timestamp;
}

void EKFManagement::ekf_update(
    const std::map<SensorType, std::set<int>> &update_list) {
  if (update_list.empty()) {
    return;
  }

  Eigen::MatrixXd P;
  Eigen::VectorXd x;
  std::map<SensorType, std::map<int, int>> ekf_lm_pos;
  int n_state = get_state_size(ekf_lm_pos);
  construct_x_and_P(x, P, n_state, ekf_lm_pos);
  const int state_size = P.rows();
  const int residual_size = get_residual_size(update_list);

  Eigen::MatrixXd Hx = Eigen::MatrixXd::Zero(residual_size, state_size);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);
  Eigen::VectorXd residual = Eigen::VectorXd::Zero(residual_size);

  int pos = 0;
  std::cout << "Update List: ";
  for (auto it_type = update_list.begin(); it_type != update_list.end();
       ++it_type) {
    const SensorType &semantic_type = it_type->first;
    for (auto it = it_type->second.begin(); it != it_type->second.end(); ++it) {
      const int &landmark_id = *it;
      std::cout << landmark_id << " ";
      Eigen::VectorXd r;
      Eigen::MatrixXd Jacobian_vehicle, Jacobian_landmark;
      SemanticMap::GetInstance()
          .GetLandmark(semantic_type, landmark_id)
          ->GetLatestResidualAndJacobian(_vehicle_x, r, Jacobian_vehicle,
                                         Jacobian_landmark);
      int residual_size, landmark_size;
      switch (semantic_type) {
        case SEMANTIC_TYPE_PARKING_SLOT:
          residual_size = RESIDUAL_PARKING_SLOT_SIZE;
          landmark_size = STATE_PARKING_SLOT_SIZE;
          break;

        default:
          break;
      }

      residual.segment(pos, residual_size) = r;
      R.block(pos, pos, residual_size, residual_size) =
          SemanticMap::GetInstance()
              .GetLandmark(semantic_type, landmark_id)
              ->GetLatestMea()
              ->GetMeasurementNosise();

      Hx.block(pos, 0, residual_size, STATE_VEHICLE_SIZE) = Jacobian_vehicle;
      int lm_pos = ekf_lm_pos.at(semantic_type).at(landmark_id);
      Hx.block(pos, lm_pos, residual_size, landmark_size) = Jacobian_landmark;

      pos += residual_size;
    }
  }

  // residual *= -1.0;
  std::cout << std::endl;
  // std::cout << "Residual:\n" << residual.transpose() << std::endl;
  // std::cout << "#######################\n";
  // std::cout << "Hx\n" << Hx << std::endl;
  // std::cout << "#######################\n";
  // std::cout << "x----:\n" << x.transpose() << std::endl;
  // std::cout << "P-----\n";
  // std::cout << P << std::endl;
  // std::cout << "-----\n";

  // std::cout << R << std::endl;
  if (residual.size() > 0) {
    Eigen::MatrixXd S = Hx * P * Hx.transpose() + R;
    Eigen::MatrixXd Sinv = S.inverse();
    // std::cout << "Identity\n";
    // std::cout << S * Sinv << std::endl;
    Eigen::MatrixXd K = P * Hx.transpose() * Sinv;
    x = x + K * residual;
    P = P - K * (Hx * P * Hx.transpose() + R) * K.transpose();
    P = (P + P.transpose()) * 0.5;
    // std::cout << "x+++++++:\n" << x.transpose() << std::endl;
    // std::cout << "P+++++\n";
    // std::cout << P << std::endl;
    // MatrixPlot::GetInstance().PlotCovarianceMatrix(P);

    update_mean_and_cov(x, P, ekf_lm_pos);
  }
}

int EKFManagement::get_residual_size(
    const std::map<SensorType, std::set<int>> &update_list) {
  int residual_size = 0;
  for (auto it = update_list.begin(); it != update_list.end(); ++it) {
    const SensorType &semantic_type = it->first;
    int size;
    switch (semantic_type) {
      case SEMANTIC_TYPE_PARKING_SLOT:
        size = STATE_PARKING_SLOT_SIZE;
        break;

      default:
        break;
    }

    residual_size += size * it->second.size();
  }

  return residual_size;
}

void EKFManagement::update_mean_and_cov(
    const Eigen::VectorXd &x, const Eigen::MatrixXd &P,
    const std::map<SensorType, std::map<int, int>> &ekf_lm_pos) {
  for (auto it_type = ekf_lm_pos.begin(); it_type != ekf_lm_pos.end();
       ++it_type) {
    const auto &semantic_type = it_type->first;
    for (auto it = it_type->second.begin(); it != it_type->second.end(); ++it) {
      const int semantic_id = it->first;
      const int pos = it->second;

      CrossCorrelationId correlation_id(semantic_type, semantic_id);
      int size;
      switch (semantic_type) {
        case SEMANTIC_TYPE_PARKING_SLOT:
          size = STATE_PARKING_SLOT_SIZE;
          break;

        default:
          break;
      }
      _state_lm_cross_correlation.at(correlation_id) =
          P.block(0, pos, STATE_VEHICLE_SIZE, size);

      SemanticMap::GetInstance().SetLandmarkCov(semantic_type, semantic_id,
                                                P.block(pos, pos, size, size));

      SemanticMap::GetInstance().SetLandmarkMean(semantic_type, semantic_id,
                                                 x.segment(pos, size));
    }
  }

  for (auto it_type0 = _lm_state_list.begin(); it_type0 != _lm_state_list.end();
       ++it_type0) {
    const auto &type0 = it_type0->first;
    const auto &set0 = it_type0->second;
    for (auto it_i = set0.begin(); it_i != set0.end(); ++it_i) {
      const int &id0 = *it_i;

      auto it_type1 = it_type0;
      for (; it_type1 != _lm_state_list.end(); ++it_type1) {
        const auto &type1 = it_type1->first;
        const auto &set1 = it_type1->second;

        auto j_start = (it_type0 == it_type1) ? it_i : set1.begin();
        for (auto it_j = j_start; it_j != set1.end(); ++it_j) {
          if (it_type0 != it_type1 || it_i != it_j) {
            const int &id1 = *it_j;
            int pos0 = ekf_lm_pos.at(type0).at(id0);
            int pos1 = ekf_lm_pos.at(type1).at(id1);
            int size0, size1;
            switch (type0) {
              case SEMANTIC_TYPE_PARKING_SLOT:
                size0 = STATE_PARKING_SLOT_SIZE;
                break;

              default:
                break;
            }

            switch (type1) {
              case SEMANTIC_TYPE_PARKING_SLOT:
                size1 = STATE_PARKING_SLOT_SIZE;
                break;

              default:
                break;
            }

            Eigen::MatrixXd correlation = P.block(pos0, pos1, size0, size1);
            CrossCorrelationId correlation_id0(type0, id0);
            CrossCorrelationId correlation_id1(type1, id1);

            set_cross_correlation(correlation_id0, correlation_id1,
                                  correlation);
          }
        }
      }
    }
  }
}

int EKFManagement::get_state_size(
    std::map<SensorType, std::map<int, int>> &ekf_lm_pos) {
  ekf_lm_pos.clear();
  int n_state_size = STATE_VEHICLE_SIZE;
  for (auto it = _lm_state_list.begin(); it != _lm_state_list.end(); ++it) {
    for (auto itt = it->second.begin(); itt != it->second.end(); ++itt) {
      switch (it->first) {
        case SEMANTIC_TYPE_PARKING_SLOT: {
          const int id = *itt;
          ekf_lm_pos[it->first].insert({id, n_state_size});
          n_state_size += STATE_PARKING_SLOT_SIZE;
          break;
        }

        default:
          break;
      }
    }
  }

  return n_state_size;
}

void EKFManagement::construct_x_and_P(
    Eigen::VectorXd &x, Eigen::MatrixXd &P, const int &state_size,
    const std::map<SensorType, std::map<int, int>> &ekf_lm_pos) {
  int n_state_size = state_size;
  x = Eigen::VectorXd::Zero(n_state_size);
  x.head(3) = _vehicle_x;

  P = Eigen::MatrixXd::Zero(n_state_size, n_state_size);
  P.topLeftCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) = _vehicle_P;

  for (auto it_type0 = _lm_state_list.begin(); it_type0 != _lm_state_list.end();
       ++it_type0) {
    const auto &type0 = it_type0->first;
    const auto &set0 = it_type0->second;

    for (auto it_i = set0.begin(); it_i != set0.end(); ++it_i) {
      const int &id0 = *it_i;
      auto it_type1 = it_type0;
      int pos0 = ekf_lm_pos.at(type0).at(id0);
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

      // cross_correlation with vehicle
      CrossCorrelationId correlation_id0(type0, id0);

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

        auto j_start = (it_type0 == it_type1) ? it_i : set1.begin();
        for (auto it_j = j_start; it_j != set1.end(); ++it_j) {
          const int &id1 = *it_j;
          int pos1 = ekf_lm_pos.at(type1).at(id1);

          CrossCorrelationId correlation_id1(type1, id1);

          if (it_type0 == it_type1 && it_i == it_j) {
            P.block(pos0, pos0, size0, size0) = auto_correlation;
            x.segment(pos0, size0) = SemanticMap::GetInstance()
                                         .GetLandmark(type0, id0)
                                         ->GetVectorizedData();
            P.block(0, pos0, STATE_VEHICLE_SIZE, size0) =
                _state_lm_cross_correlation.at(correlation_id0);
            P.block(pos0, 0, size0, STATE_VEHICLE_SIZE) =
                P.block(0, pos0, STATE_VEHICLE_SIZE, size0).transpose();
          }

          if (it_type0 != it_type1 || it_i != it_j) {
            // initialize cross-correlation
            // std::cout << "cross correlation: " << id0 << " " << id1
            //           << std::endl;
            bool transpose = false;
            Eigen::MatrixXd correlation = get_cross_correlation(
                correlation_id0, correlation_id1, transpose);
            if (transpose) {
              correlation = correlation.transpose();
            }
            P.block(pos0, pos1, size0, size1) = correlation;
            P.block(pos1, pos0, size1, size0) =
                P.block(pos0, pos1, size0, size1).transpose();
            // std::cout << P.block(pos0, pos1, size0, size1) << std::endl;
          }
        }
      }

      pos0 += size0;
    }
  }
}

void EKFManagement::aug_update_covariance(const SensorType &type, const int &id,
                                          const Eigen::MatrixXd &Jx) {
  Eigen::MatrixXd P;
  Eigen::VectorXd x;
  std::map<SensorType, std::map<int, int>> ekf_lm_pos;
  int n_state_size = get_state_size(ekf_lm_pos);
  construct_x_and_P(x, P, n_state_size, ekf_lm_pos);
  // MatrixPlot::GetInstance().PlotCovarianceMatrix(P);
  // std::cout << P << std::endl;
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

  P.topRightCorner(rows, size) =
      P.topLeftCorner(rows, STATE_VEHICLE_SIZE) * Jx.transpose();
  P.bottomLeftCorner(size, rows) = P.topRightCorner(rows, size).transpose();

  P.bottomRightCorner(size, size) =
      Jx * P.topLeftCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) *
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
  _state_lm_cross_correlation[correlation_id] =
      P.topRightCorner(STATE_VEHICLE_SIZE, size);
  _lm_state_list[type].insert(id);
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
              semantic_type, landmark_id, _vehicle_x, _vehicle_P, Jx);
          std::cout << "Augmenting: id " << landmark_id << std::endl;
          aug_update_covariance(semantic_type, landmark_id, Jx);
          break;
        }

        default:
          break;
      }
    }
  }
}
void EKFManagement::state_marginalization(
    const std::map<SensorType, std::set<int>> &marginalization_list) {
  for (auto it_type = marginalization_list.begin();
       it_type != marginalization_list.end(); ++it_type) {
    const auto &type = it_type->first;
    for (auto it_id = it_type->second.begin(); it_id != it_type->second.end();
         ++it_id) {
      const int &id = *it_id;
      _lm_state_list.at(type).erase(id);
      SemanticMap::GetInstance().MarginLandmark(type, id);
    }
  }
}

CrossCorrelationKey EKFManagement::make_lm_cross_correlation_key(
    const CrossCorrelationId &id0, const CrossCorrelationId &id1) {
  return CrossCorrelationKey(id0, id1);
}

void EKFManagement::set_cross_correlation(const CrossCorrelationId &id0,
                                          const CrossCorrelationId &id1,
                                          const Eigen::MatrixXd &correlation) {
  // correlation [size0 x size1]  P_01
  CrossCorrelationKey key = make_lm_cross_correlation_key(id0, id1);
  if (id0.type == key.first.type && id0.id == key.first.id) {
    _lm_cross_correlation[key] = correlation;
  } else {
    _lm_cross_correlation[key] = correlation.transpose();
  }
}

Eigen::MatrixXd EKFManagement::get_cross_correlation(
    const CrossCorrelationId &id0, const CrossCorrelationId &id1,
    bool &transpose) {
  CrossCorrelationKey key = make_lm_cross_correlation_key(id0, id1);
  if (id0.type == key.first.type && id0.id == key.first.id) {
    transpose = false;
    return _lm_cross_correlation.at(key);
  } else {
    transpose = true;
    return _lm_cross_correlation.at(key).transpose();
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

void EKFManagement::Propagate(const double timestamp_d, const double v,
                              const double w) {
  _vehicle_w = w;
  _vehicle_v = v;

  if (!_initialized) {
    _initialized = true;
    _ts = timestamp_d;
    return;
  }

  double dt = timestamp_d - _ts;

  std::cout << " -----Propagate to " << std::setprecision(20) << timestamp_d
            << " " << v << " " << w << " " << dt << std::endl;

  Eigen::VectorXd x_full;
  Eigen::MatrixXd P_full;
  std::map<SensorType, std::map<int, int>> ekf_lm_pos;
  int n_state = get_state_size(ekf_lm_pos);
  construct_x_and_P(x_full, P_full, n_state, ekf_lm_pos);

  Eigen::Vector2d twb = _vehicle_x.head(2);
  double yaw = _vehicle_x[2];
  Eigen::Rotation2Dd rot(yaw);
  Eigen::Matrix2d Rwb = rot.toRotationMatrix();
  Eigen::Vector2d dir = Rwb.col(0);

  Eigen::MatrixXd Fx = Eigen::MatrixXd::Identity(3, 3);
  Eigen::Vector2d d_dir_yaw(-std::sin(yaw), std::cos(yaw));
  Fx.topRightCorner(2, 1) = d_dir_yaw * v * dt;

  Eigen::MatrixXd Fn = Eigen::MatrixXd::Zero(3, 2);
  Fn.topLeftCorner(2, 1) = dir * dt;
  Fn(2, 1) = dt;

  twb += dir * v * dt;
  yaw += dt * w;

  Eigen::MatrixXd P =
      Fx * _vehicle_P * Fx.transpose() + Fn * _N * Fn.transpose();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(STATE_VEHICLE_SIZE);
  x.head(2) = twb;
  x[2] = yaw;

  x_full.head(3) = x;
  P_full.topLeftCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) = P;

  if (n_state > STATE_VEHICLE_SIZE) {
    P_full.topRightCorner(STATE_VEHICLE_SIZE, n_state - STATE_VEHICLE_SIZE) =
        Fx *
        P_full.topRightCorner(STATE_VEHICLE_SIZE, n_state - STATE_VEHICLE_SIZE);
    P_full.bottomLeftCorner(n_state - STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) =
        P_full.topRightCorner(STATE_VEHICLE_SIZE, n_state - STATE_VEHICLE_SIZE)
            .transpose();
  }

  update_mean_and_cov(x_full, P_full, ekf_lm_pos);

  _vehicle_x = x;
  _vehicle_P = P;
  _ts = timestamp_d;
  // std::cout << " ++++++Propagate: \n" << P_vehicle1 << std::endl;
  // std::cout << "===================\n";
}

}  // namespace apa_slam