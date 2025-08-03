/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/sliding_window.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Friday, May 30th 2025, 5:09:43 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "sliding_window.h"

namespace apa_slam {
SlidingWindow::SlidingWindow() {}

void SlidingWindow::Init() {
  double noise_v =
      ApaParameters::GetInstance().GetEstimatorParamters().odo_velocity_noise;
  double noise_w = ApaParameters::GetInstance()
                       .GetEstimatorParamters()
                       .odo_angular_velocity_noise;
  _N = Eigen::MatrixXd::Zero(NOISE_ODO_SIZE, NOISE_ODO_SIZE);
  _N.diagonal()[0] = noise_v * noise_v;
  _N.diagonal()[1] = noise_w * noise_w;
}

void SlidingWindow::Reset() {
  _sl_timestamp.clear();
  _sl_P.clear();
  _sl_pose.clear();
  _state_landmark.clear();
  _window_cross_correlation.clear();
  _window_lm_cross_correlation.clear();
  _initialized = false;
}

int SlidingWindow::GetCurWindowSz() { return _sl_timestamp.size(); }

bool SlidingWindow::GetSlidingWindowStatus(const int id, long long &timestamp,
                                           Eigen::VectorXd &x,
                                           Eigen::MatrixXd &P) {
  if (id > this->GetCurWindowSz() - 1) {
    std::cout << "Fatal Error, Window Id Wrong\n";
    return false;
  }

  timestamp = _sl_timestamp.at(id);
  x = _sl_pose.at(id);
  P = _sl_P.at(id);

  return true;
}

bool SlidingWindow::Initialized() { return _initialized; }

void SlidingWindow::Propagate(
    const long long ts,
    const std::vector<std::pair<long long, Eigen::VectorXd>> &odo_meas) {
  if (this->GetCurWindowSz() == 0) {
    std::cout << "Fatal Error, this should be done with at lease one window\n";
  }

  Eigen::VectorXd x;
  Eigen::MatrixXd P;
  std::map<SensorType, std::map<int, int>> lm_pos;
  int state_sz = get_state_size(lm_pos);
  construct_x_and_P(x, P, state_sz, lm_pos);

  Eigen::MatrixXd P_aug = P;
  P_aug.conservativeResize(P.rows() + STATE_VEHICLE_SIZE,
                           P.cols() + STATE_VEHICLE_SIZE);
  P_aug.bottomRightCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) =
      _sl_P.back();
  P_aug.bottomLeftCorner(STATE_VEHICLE_SIZE, P.cols()) =
      P.topRows(STATE_VEHICLE_SIZE);
  P_aug.topRightCorner(P.rows(), STATE_VEHICLE_SIZE) =
      P.leftCols(STATE_VEHICLE_SIZE);

  // std::cout << "P_aug::::::::::::::::::::::::::\n" << P_aug << std::endl;

  long long timestamp_cur = _sl_timestamp.back();
  Eigen::VectorXd x_cur = _sl_pose.back();

  double time_scale =
      ApaParameters::GetInstance().GetEstimatorParamters().time_scale;
  for (size_t i = 0; i < odo_meas.size(); ++i) {
    double dt =
        static_cast<double>(odo_meas.at(i).first - timestamp_cur) * time_scale;
    double v = odo_meas.at(i).second.x();
    double w = odo_meas.at(i).second.y();

    Eigen::Vector2d twb = x_cur.head(2);
    double yaw = x_cur[2];
    Eigen::Rotation2Dd rot(yaw);
    Eigen::Matrix2d Rwb = rot.toRotationMatrix();
    Eigen::Vector2d dir = Rwb.col(0);

    Eigen::MatrixXd Fx = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Vector2d d_dir_yaw(-std::sin(yaw), std::cos(yaw));
    Fx.topRightCorner(2, 1) = d_dir_yaw * v * dt;

    Eigen::MatrixXd Fn = Eigen::MatrixXd::Zero(3, 2);
    Fn.topLeftCorner(2, 1) = dir * dt;
    Fn(2, 1) = dt;

    Eigen::MatrixXd P_newest =
        Fx * P_aug.bottomRightCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) *
            Fx.transpose() +
        Fn * _N * Fn.transpose();

    Eigen::MatrixXd correlation_newest =
        P_aug.topRightCorner(P.rows(), STATE_VEHICLE_SIZE) * Fx.transpose();

    P_aug.bottomRightCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) = P_newest;
    P_aug.topRightCorner(P.rows(), STATE_VEHICLE_SIZE) = correlation_newest;
    P_aug.bottomLeftCorner(STATE_VEHICLE_SIZE, P.cols()) =
        P_aug.topRightCorner(P.rows(), STATE_VEHICLE_SIZE).transpose();

    twb += dir * v * dt;
    yaw += dt * w;

    x_cur.head(2) = twb;
    x_cur[2] = yaw;
    timestamp_cur = odo_meas.at(i).first;
  }

  if (this->GetCurWindowSz() ==
      ApaParameters::GetInstance().GetEstimatorParamters().window_size) {
    P_aug = P_aug.bottomRightCorner(P_aug.rows() - STATE_VEHICLE_SIZE,
                                    P_aug.cols() - STATE_VEHICLE_SIZE);
  }

  Eigen::MatrixXd PP = Eigen::MatrixXd::Zero(P_aug.rows(), P_aug.cols());
  PP.topLeftCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) =
      P_aug.bottomRightCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE);
  PP.bottomRightCorner(PP.rows() - STATE_VEHICLE_SIZE,
                       PP.cols() - STATE_VEHICLE_SIZE) =
      P_aug.topLeftCorner(PP.rows() - STATE_VEHICLE_SIZE,
                          PP.cols() - STATE_VEHICLE_SIZE);
  PP.topRightCorner(STATE_VEHICLE_SIZE, PP.cols() - STATE_VEHICLE_SIZE) =
      P_aug.bottomLeftCorner(STATE_VEHICLE_SIZE,
                             PP.cols() - STATE_VEHICLE_SIZE);
  PP.bottomLeftCorner(PP.rows() - STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) =
      P_aug.topRightCorner(PP.rows() - STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE);

  if (this->GetCurWindowSz() <
      ApaParameters::GetInstance().GetEstimatorParamters().window_size) {
    for (auto it = lm_pos.begin(); it != lm_pos.end(); ++it) {
      for (auto itt = it->second.begin(); itt != it->second.end(); ++itt) {
        itt->second += STATE_VEHICLE_SIZE;
      }
    }
  }

  refresh_propagate_window_status(ts, x_cur, PP, lm_pos);

  data_check();
} // namespace apa_slam

void SlidingWindow::construct_x_and_P(
    Eigen::VectorXd &x, Eigen::MatrixXd &P, const int &state_sz,
    const std::map<SensorType, std::map<int, int>> &ekf_lm_pos) {
  P = Eigen::MatrixXd::Zero(state_sz, state_sz);
  x = Eigen::VectorXd::Zero(state_sz);

  for (int i = 0; i < this->GetCurWindowSz(); ++i) {
    int start_pos = STATE_VEHICLE_SIZE * (this->GetCurWindowSz() - i - 1);
    x.segment(start_pos, STATE_VEHICLE_SIZE) = _sl_pose[i];
    P.block(start_pos, start_pos, STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) =
        _sl_P[i];
  } // window auto correlation

  // window cross correlation
  for (int i = 0; i < this->GetCurWindowSz() - 1; ++i) {
    for (int j = i + 1; j < this->GetCurWindowSz(); ++j) {
      Eigen::MatrixXd correlation = get_window_cross_correlation(i, j);
      int pos_i = STATE_VEHICLE_SIZE * (this->GetCurWindowSz() - i - 1);
      int pos_j = STATE_VEHICLE_SIZE * (this->GetCurWindowSz() - j - 1);
      P.block(pos_i, pos_j, STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) =
          get_window_cross_correlation(i, j);
      P.block(pos_j, pos_i, STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) =
          P.block(pos_i, pos_j, STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE)
              .transpose();
    }
  }

  // lm auto correlation
  std::vector<std::tuple<SensorType, int, int>> ekf_lm_pos_vector;
  for (auto it_lm = ekf_lm_pos.begin(); it_lm != ekf_lm_pos.end(); ++it_lm) {
    const auto &type = it_lm->first;
    int lm_state_size;
    switch (type) {
    case SensorType::SEMANTIC_TYPE_PARKING_SLOT:
      lm_state_size = STATE_PARKING_SLOT_SIZE;
      break;

    default:
      break;
    }
    for (auto it = it_lm->second.begin(); it != it_lm->second.end(); ++it) {
      int lm_id = it->first;
      int state_pos = it->second;
      x.segment(state_pos, lm_state_size) = SemanticMap::GetInstance()
                                                .GetLandmark(type, lm_id)
                                                ->GetVectorizedData();
      P.block(state_pos, state_pos, lm_state_size, lm_state_size) =
          SemanticMap::GetInstance().GetLandmark(type, lm_id)->GetCov();
      if (only_localization) {
        P.block(state_pos, state_pos, lm_state_size, lm_state_size).setZero();
      }
      ekf_lm_pos_vector.push_back({type, lm_id, state_pos});
    }
  }

  // lm cross correlation
  if (ekf_lm_pos_vector.size() >= 2) {
    for (size_t i = 0; i < ekf_lm_pos_vector.size() - 1; ++i) {
      auto type0 = std::get<0>(ekf_lm_pos_vector.at(i));
      auto lmid0 = std::get<1>(ekf_lm_pos_vector.at(i));
      auto pos0 = std::get<2>(ekf_lm_pos_vector.at(i));
      CrossCorrelationId id0(type0, lmid0);
      int state_sz0;
      switch (type0) {
      case SensorType::SEMANTIC_TYPE_PARKING_SLOT:
        state_sz0 = STATE_PARKING_SLOT_SIZE;
        break;

      default:
        break;
      }

      for (size_t j = i + 1; j < ekf_lm_pos_vector.size(); ++j) {
        auto type1 = std::get<0>(ekf_lm_pos_vector.at(j));
        auto lmid1 = std::get<1>(ekf_lm_pos_vector.at(j));
        auto pos1 = std::get<2>(ekf_lm_pos_vector.at(j));
        CrossCorrelationId id1(type1, lmid1);
        int state_sz1;
        switch (type1) {
        case SensorType::SEMANTIC_TYPE_PARKING_SLOT:
          state_sz1 = STATE_PARKING_SLOT_SIZE;
          break;

        default:
          break;
        }

        P.block(pos0, pos1, state_sz0, state_sz1) =
            this->get_landmark_cross_correlation(id0, id1);
        P.block(pos1, pos0, state_sz1, state_sz0) =
            P.block(pos0, pos1, state_sz0, state_sz1).transpose();

        if (only_localization) {
          P.block(pos0, pos1, state_sz0, state_sz1).setZero();
          P.block(pos1, pos0, state_sz1, state_sz0).setZero();
        }
      }
    }
  }

  // window lm cross correlation

  for (int i = 0; i < this->GetCurWindowSz(); ++i) {
    int window_pos = STATE_VEHICLE_SIZE * (this->GetCurWindowSz() - i - 1);
    for (size_t j = 0; j < ekf_lm_pos_vector.size(); ++j) {
      auto type = std::get<0>(ekf_lm_pos_vector.at(j));
      auto landmark_id = std::get<1>(ekf_lm_pos_vector.at(j));
      auto lm_pos = std::get<2>(ekf_lm_pos_vector.at(j));
      int lm_state_sz;
      switch (type) {
      case SensorType::SEMANTIC_TYPE_PARKING_SLOT:
        lm_state_sz = STATE_PARKING_SLOT_SIZE;
        break;

      default:
        break;
      }

      Eigen::MatrixXd correlation =
          get_window_landmark_cross_correlation(i, type, landmark_id);
      // std::cout << "get_window_landmark_cross_correlation " << i << " " <<
      // landmark_id << std::endl;
      P.block(window_pos, lm_pos, STATE_VEHICLE_SIZE, lm_state_sz) =
          correlation;
      P.block(lm_pos, window_pos, lm_state_sz, STATE_VEHICLE_SIZE) =
          correlation.transpose();

      if (only_localization) {
        P.block(window_pos, lm_pos, STATE_VEHICLE_SIZE, lm_state_sz).setZero();
        P.block(lm_pos, window_pos, lm_state_sz, STATE_VEHICLE_SIZE).setZero();
      }
    }
  }
}

long long SlidingWindow::GetSlwTimestamp(const int id) {
  return _sl_timestamp.at(id);
}

void SlidingWindow::InitializeSlw(const long long ts, const Eigen::VectorXd &x,
                                  const Eigen::MatrixXd &P) {
  _sl_pose.push_back(x);
  _sl_timestamp.push_back(ts);
  _sl_P.push_back(P);

  _initialized = true;
}

int SlidingWindow::get_residual_sz(
    const std::vector<std::map<SensorType, std::vector<int>>> &sw_lm_list) {
  int sz = 0;
  for (size_t i = 0; i < sw_lm_list.size(); ++i) {
    for (auto it_type = sw_lm_list.at(i).begin();
         it_type != sw_lm_list.at(i).end(); ++it_type) {
      const auto &type = it_type->first;
      for (size_t j = 0; j < it_type->second.size(); ++j) {
        int lm_id = it_type->second.at(j);
        switch (type) {
        case SensorType::SEMANTIC_TYPE_PARKING_SLOT:
          sz += STATE_PARKING_SLOT_SIZE;
          break;

        default:
          break;
        }
      }
    }
  }

  return sz;
}

void SlidingWindow::UpdateEKF(
    const Eigen::VectorXd &x, const Eigen::MatrixXd &P,
    const std::map<SensorType, std::map<int, int>> &ekf_lm_pos,
    const std::map<SensorType, std::set<int>> &marginalization_list) {
  for (size_t i = 0; i < this->GetCurWindowSz(); ++i) {
    int win_pos = STATE_VEHICLE_SIZE * (this->GetCurWindowSz() - i - 1);
    _sl_pose[i] = x.segment(win_pos, STATE_VEHICLE_SIZE);
    _sl_P[i] =
        P.block(win_pos, win_pos, STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE);
  }

  refresh_landmark_cross_correlation(P, ekf_lm_pos);
  refresh_window_cross_correlation(P);
  refresh_window_landmark_cross_correlation(P, ekf_lm_pos);

  for (auto it = ekf_lm_pos.begin(); it != ekf_lm_pos.end(); ++it) {
    for (auto itt = it->second.begin(); itt != it->second.end(); ++itt) {
      auto type = it->first;
      auto lm_id = itt->first;
      auto pos = itt->second;
      SemanticMap::GetInstance().SetLandmarkMean(type, lm_id,
                                                 x.segment(pos, 4));
      // SemanticMap::GetInstance().SetLandmarkCov(type, lm_id,
      //                                           P.block(pos, pos, 4, 4));
    }

    marginalization(marginalization_list);
  }

  data_check();
}

void SlidingWindow::ConstructEKF(
    Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::VectorXd &residual,
    Eigen::MatrixXd &H, Eigen::MatrixXd &R,
    std::map<SensorType, std::map<int, int>> &ekf_lm_pos,
    std::map<SensorType, std::set<int>> &marginalization_list) {
  std::vector<std::map<SensorType, std::vector<int>>> sw_lm_list;
  std::map<SensorType, std::set<int>> margin_list;
  SemanticMap::GetInstance().GetSlidingWindowEKFDataList(
      _sl_timestamp, sw_lm_list, margin_list);

  // std::vector<std::map<SensorType, std::vector<int>>> sw_lm_list_copy;
  // sw_lm_list_copy.resize(sw_lm_list.size());
  // for (size_t i = 0; i < sw_lm_list.size(); ++i) {
  //   if (!sw_lm_list.at(i).empty()) {
  //     for (size_t j = 0;
  //          j <
  //          sw_lm_list.at(i).at(SensorType::SEMANTIC_TYPE_PARKING_SLOT).size();
  //          ++j) {
  //       if
  //       (sw_lm_list.at(i).at(SensorType::SEMANTIC_TYPE_PARKING_SLOT).at(j)
  //       ==
  //           0) {
  //         sw_lm_list_copy.at(i)[SensorType::SEMANTIC_TYPE_PARKING_SLOT]
  //             .push_back(0);
  //       }
  //     }
  //     std::cout
  //         << "Window Id: " << i << " Update Landmark Size: "
  //         <<
  //         sw_lm_list.at(i).at(SensorType::SEMANTIC_TYPE_PARKING_SLOT).size()
  //         << std::endl;
  //   }
  // }

  // sw_lm_list_copy = sw_lm_list;

  initialize_landmark(sw_lm_list);
  if (only_localization) {
    margin_list.clear();
  }
  if (!marginalization_list.empty()) {
    for (auto it = marginalization_list.begin()->second.begin();
         it != marginalization_list.begin()->second.end(); ++it) {
      std::cout << "Marginalization: " << *it << std::endl;
    }
  }

  marginalization_list = margin_list;

  int state_sz = this->get_state_size(ekf_lm_pos);
  construct_x_and_P(x, P, state_sz, ekf_lm_pos);

  // apa_slam::MatrixPlot::GetInstance().PlotCovarianceMatrix(P);
  // std::cout << P << std::endl;
  // std::cout << "======================\n";
  // getchar();

  int residual_sz = get_residual_sz(sw_lm_list);
  residual = Eigen::VectorXd::Zero(residual_sz);
  H = Eigen::MatrixXd::Zero(residual_sz, state_sz);
  R = Eigen::MatrixXd::Zero(residual_sz, residual_sz);

  int residual_pos = 0;
  for (size_t i = 0; i < sw_lm_list.size(); ++i) {
    int window_pos = STATE_VEHICLE_SIZE * (this->GetCurWindowSz() - i - 1);
    for (auto it_type = sw_lm_list.at(i).begin();
         it_type != sw_lm_list.at(i).end(); ++it_type) {
      const auto &type = it_type->first;
      for (size_t j = 0; j < it_type->second.size(); ++j) {
        int lm_id = it_type->second.at(j);
        int lm_pos = ekf_lm_pos.at(type).at(lm_id);
        SemanticMea::Ptr mea;
        SemanticMap::GetInstance()
            .GetLandmark(type, lm_id)
            ->GetMea(_sl_timestamp.at(i), mea);
        Eigen::VectorXd r;
        Eigen::MatrixXd J_window, J_landmark;
        SemanticMap::GetInstance()
            .GetLandmark(type, lm_id)
            ->GetResidualAndJacobian(mea, _sl_pose.at(i), r, J_window,
                                     J_landmark);
        // J_landmark.setZero();

        // std::cout << "residual: " << r.transpose() << " winid: " << i
        //           << " lmid: " << lm_id << std::endl;
        residual.segment(residual_pos, r.size()) = r;
        R.block(residual_pos, residual_pos, r.size(), r.size()) =
            SemanticMap::GetInstance()
                .GetLandmark(type, lm_id)
                ->GetLatestMea()
                ->GetMeasurementNosise();

        H.block(residual_pos, window_pos, r.size(), STATE_VEHICLE_SIZE) =
            J_window;
        H.block(residual_pos, lm_pos, r.size(), STATE_PARKING_SLOT_SIZE) =
            J_landmark;
        bool is_tar = false;
        if (type == SensorType::SEMANTIC_TYPE_PARKING_SLOT) {
          auto ps_ptr = std::dynamic_pointer_cast<ParkingSlotLandmark>(
              SemanticMap::GetInstance().GetLandmark(type, lm_id));
          if (ps_ptr->IsTarget()) {
            is_tar = true;
          }
        }
        if (only_localization) {
          H.block(residual_pos, lm_pos, r.size(), STATE_PARKING_SLOT_SIZE)
              .setZero();
        }

        // if (only_localization && !is_tar) {
        //   H.block(residual_pos, window_pos, r.size(), STATE_VEHICLE_SIZE)
        //       .setZero();
        // }

        if (only_localization && is_tar) {
          // H.block(residual_pos, window_pos, r.size(),
          // STATE_VEHICLE_SIZE).setZero();
          R.block(residual_pos, residual_pos, r.size(), r.size()) *= 0.01;
        }

        residual_pos += r.size();
      }
    }
  }

  // std::cout << "H\n" << H << std::endl;
  // std::cout << "#############################\n";
  // std::cout << "R\n" << residual.transpose() << std::endl;
  // std::cout << "#############################\n";

#ifdef ENABLE_OPENGL
  {
    std::unique_lock<std::shared_mutex> lock(gl_slw.mutex);
    gl_slw.sl_pose.clear();
    gl_slw.sl_meas.clear();
    gl_slw.sl_pose.resize(this->GetCurWindowSz());
    gl_slw.sl_meas.resize(this->GetCurWindowSz());

    for (size_t i = 0; i < this->GetCurWindowSz(); ++i) {
      gl_slw.sl_pose.at(i) = _sl_pose.at(i);

      const auto &timestamp = _sl_timestamp.at(i);
      Eigen::Vector2d twb = _sl_pose.at(i).head(2);
      double yaw = _sl_pose.at(i).z();
      Eigen::Rotation2Dd rot(yaw);
      Eigen::Matrix2d Rwb = rot.toRotationMatrix();
      if (!sw_lm_list.at(i).empty()) {
        for (size_t j = 0;
             j !=
             sw_lm_list.at(i).at(SensorType::SEMANTIC_TYPE_PARKING_SLOT).size();
             ++j) {
          int landmark_id =
              sw_lm_list.at(i).at(SensorType::SEMANTIC_TYPE_PARKING_SLOT).at(j);
          apa_slam::SemanticMea::Ptr mea;
          if (SemanticMap::GetInstance()
                  .GetLandmark(SensorType::SEMANTIC_TYPE_PARKING_SLOT,
                               landmark_id)
                  ->GetMea(timestamp, mea)) {
            Eigen::VectorXd mea_data = mea->GetVectorizedData();
            Eigen::Vector2d mea0_w = Rwb * mea_data.head(2) + twb;
            Eigen::Vector2d mea1_w = Rwb * mea_data.tail(2) + twb;
            gl_slw.sl_meas.at(i).push_back(mea0_w);
            gl_slw.sl_meas.at(i).push_back(mea1_w);
          }
        }
      }
    }
  }

#endif
}

SlidingWindow &SlidingWindow::GetInstance() {
  static SlidingWindow instance;
  return instance;
}

void SlidingWindow::set_window_landmark_cross_correlation(
    const int &window_id, const SensorType &type, const int &lm_id,
    const Eigen::MatrixXd &correlation) {
  CrossCorrelationId id(type, lm_id);
  _window_lm_cross_correlation[window_id].insert({id, correlation});
}

Eigen::MatrixXd SlidingWindow::get_window_landmark_cross_correlation(
    const int &window_id, const SensorType &type, const int &lm_id) {
  auto id = CrossCorrelationId(type, lm_id);
  Eigen::MatrixXd correlation = _window_lm_cross_correlation.at(window_id)[id];

  return correlation;
}

void SlidingWindow::refresh_window_landmark_cross_correlation(
    const Eigen::MatrixXd &P,
    const std::map<SensorType, std::map<int, int>> &ekf_lm_pos) {
  _window_lm_cross_correlation.clear();
  _window_lm_cross_correlation.resize(this->GetCurWindowSz());

  for (size_t i = 0; i < this->GetCurWindowSz(); ++i) {
    int pos_window = STATE_VEHICLE_SIZE * (this->GetCurWindowSz() - i - 1);

    for (auto it_type = ekf_lm_pos.begin(); it_type != ekf_lm_pos.end();
         ++it_type) {
      const auto &type = it_type->first;
      for (auto it = it_type->second.begin(); it != it_type->second.end();
           ++it) {
        const auto &landmark_id = it->first;
        const auto &pos = it->second;
        int lm_state_sz;
        switch (type) {
        case SensorType::SEMANTIC_TYPE_PARKING_SLOT:
          lm_state_sz = STATE_PARKING_SLOT_SIZE;
          break;

        default:
          break;
        }

        Eigen::MatrixXd correlation =
            P.block(pos_window, pos, STATE_VEHICLE_SIZE, lm_state_sz);
        // std::cout << "set_window_landmark_cross_correlation " << i << " "
        // << landmark_id << std::endl;
        set_window_landmark_cross_correlation(i, type, landmark_id,
                                              correlation);
      }
    }
  }
}

void SlidingWindow::refresh_window_cross_correlation(const Eigen::MatrixXd &P) {
  _window_cross_correlation.clear();

  for (size_t i = 0; i < this->GetCurWindowSz() - 1; ++i) {
    for (size_t j = i + 1; j < this->GetCurWindowSz(); ++j) {
      int pos_i = STATE_VEHICLE_SIZE * (this->GetCurWindowSz() - i - 1);
      int pos_j = STATE_VEHICLE_SIZE * (this->GetCurWindowSz() - j - 1);

      Eigen::MatrixXd cross_correlation =
          P.block(pos_i, pos_j, STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE); // P_ij

      set_window_cross_correlation(i, j, cross_correlation);
    }
  }
}

void SlidingWindow::refresh_landmark_cross_correlation(
    const Eigen::MatrixXd &P,
    const std::map<SensorType, std::map<int, int>> &ekf_lm_pos) {
  _lm_cross_correlation.clear();
  std::vector<std::tuple<SensorType, int, int>> ekf_lm_pos_vector;
  for (auto it_type = ekf_lm_pos.begin(); it_type != ekf_lm_pos.end();
       ++it_type) {
    const auto &type = it_type->first;
    for (auto it = it_type->second.begin(); it != it_type->second.end(); ++it) {
      const auto &landmark_id = it->first;
      const auto &pos = it->second;
      ekf_lm_pos_vector.push_back({type, landmark_id, pos});
    }
  }

  if (ekf_lm_pos_vector.size() < 2) {
    return;
  }

  for (size_t i = 0; i < ekf_lm_pos_vector.size() - 1; ++i) {
    auto type0 = std::get<0>(ekf_lm_pos_vector.at(i));
    auto lmid0 = std::get<1>(ekf_lm_pos_vector.at(i));
    auto pos0 = std::get<2>(ekf_lm_pos_vector.at(i));
    CrossCorrelationId id0(type0, lmid0);
    int state_sz0;
    switch (type0) {
    case SensorType::SEMANTIC_TYPE_PARKING_SLOT:
      state_sz0 = STATE_PARKING_SLOT_SIZE;
      break;

    default:
      break;
    }

    for (size_t j = i + 1; j < ekf_lm_pos_vector.size(); ++j) {
      auto type1 = std::get<0>(ekf_lm_pos_vector.at(j));
      auto lmid1 = std::get<1>(ekf_lm_pos_vector.at(j));
      auto pos1 = std::get<2>(ekf_lm_pos_vector.at(j));
      CrossCorrelationId id1(type1, lmid1);
      int state_sz1;
      switch (type1) {
      case SensorType::SEMANTIC_TYPE_PARKING_SLOT:
        state_sz1 = STATE_PARKING_SLOT_SIZE;
        break;

      default:
        break;
      }
      Eigen::MatrixXd correlation = P.block(pos0, pos1, state_sz0, state_sz1);
      correlation.setZero();
      set_landmark_cross_correlation(id0, id1, correlation);
    }
  }
}

void SlidingWindow::marginalization(
    const std::map<SensorType, std::set<int>> &marginalization_list) {
  std::map<SensorType, std::set<int>> state_lm;
  for (auto it = _state_landmark.begin(); it != _state_landmark.end(); ++it) {
    for (auto itt = it->second.begin(); itt != it->second.end(); ++itt) {
      if (!marginalization_list.count(it->first) ||
          !marginalization_list.at(it->first).count(*itt)) {
        state_lm[it->first].insert(*itt);
      }
    }
  }
  // map
  for (auto it = marginalization_list.begin(); it != marginalization_list.end();
       ++it) {
    for (auto itt = it->second.begin(); itt != it->second.end(); ++itt) {
      SemanticMap::GetInstance().MarginLandmark(it->first, *itt); // map
      // private members
      auto it_lm = _lm_cross_correlation.begin();
      while (it_lm != _lm_cross_correlation.end()) {
        auto type0 = static_cast<SensorType>(it_lm->first.first.type);
        auto id0 = it_lm->first.first.id;
        auto type1 = static_cast<SensorType>(it_lm->first.second.type);
        auto id1 = it_lm->first.second.id;
        if ((type0 == it->first && id0 == *itt) ||
            (type1 == it->first && id1 == *itt)) {
          it_lm = _lm_cross_correlation.erase(it_lm);
        } else {
          ++it_lm;
        }
      }

      for (size_t i = 0; i < _window_lm_cross_correlation.size(); ++i) {
        if (_window_lm_cross_correlation.at(i).empty()) {
          continue;
        }
        auto it_w_lm = _window_lm_cross_correlation.at(i).begin();
        while (it_w_lm != _window_lm_cross_correlation.at(i).end()) {
          auto type = static_cast<SensorType>(it_w_lm->first.type);
          auto id = it_w_lm->first.id;
          if (type == it->first && id == *itt) {
            it_w_lm = _window_lm_cross_correlation.at(i).erase(it_w_lm);
          } else {
            ++it_w_lm;
          }
        }
      }
    }
  }

  _state_landmark = state_lm;
}

std::pair<int, int> SlidingWindow::make_window_correlation_key(const int id0,
                                                               const int id1) {
  if (id0 == id1) {
    std::cout << "FATAL ERROR, WINDOW ID CAN'T BE EUQUAL\n";
  }

  int x = std::min(id0, id1);
  int y = std::max(id0, id1);

  return {x, y};
}

Eigen::MatrixXd SlidingWindow::get_window_cross_correlation(const int id0,
                                                            const int id1) {
  auto key = make_window_correlation_key(id0, id1);
  if (id0 == key.first && id1 == key.second) {
    return _window_cross_correlation.at(key);
  } else {
    return _window_cross_correlation.at(key).transpose();
  }
}

CrossCorrelationKey SlidingWindow::make_landmark_cross_correlation_key(
    const CrossCorrelationId &id0, const CrossCorrelationId &id1) {
  return CrossCorrelationKey(id0, id1);
}

void SlidingWindow::set_landmark_cross_correlation(
    const CrossCorrelationId &id0, const CrossCorrelationId &id1,
    const Eigen::MatrixXd &correlation) {
  CrossCorrelationKey key = this->make_landmark_cross_correlation_key(id0, id1);
  if (id0.type == key.first.type && id0.id == key.first.id) {
    _lm_cross_correlation[key] = correlation;
  } else {
    _lm_cross_correlation[key] = correlation.transpose();
  }
}

Eigen::MatrixXd
SlidingWindow::get_landmark_cross_correlation(const CrossCorrelationId &id0,
                                              const CrossCorrelationId &id1) {
  CrossCorrelationKey key = make_landmark_cross_correlation_key(id0, id1);
  if (id0.type == key.first.type && id0.id == key.first.id) {
    return _lm_cross_correlation.at(key);
  } else {
    return _lm_cross_correlation.at(key).transpose();
  }
}

void SlidingWindow::set_window_cross_correlation(
    const int id0, const int id1, const Eigen::MatrixXd &correlation) {
  auto key = make_window_correlation_key(id0, id1);

  bool transpose = true;
  if (key.first == id0 && key.second == id1) {
    transpose = false;
  }
  _window_cross_correlation[key] =
      transpose == true ? correlation.transpose() : correlation;
}

void SlidingWindow::refresh_propagate_window_status(
    const long long &timestamp, const Eigen::VectorXd &latest_state,
    const Eigen::MatrixXd &P,
    const std::map<SensorType, std::map<int, int>> &ekf_lm_pos) {
  int slw_sz = ApaParameters::GetInstance().GetEstimatorParamters().window_size;
  int cur_win_sz = this->GetCurWindowSz();
  if (cur_win_sz < slw_sz) {
    for (size_t i = 0; i < cur_win_sz; ++i) {
      int start_pos = STATE_VEHICLE_SIZE * (this->GetCurWindowSz() - i - 1);
      _sl_P.at(i) =
          P.block(start_pos, start_pos, STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE);
    }

    _sl_P.push_back(P.topLeftCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE));
    _sl_pose.push_back(latest_state);
    _sl_timestamp.push_back(timestamp);

  } else {
    for (size_t i = 0; i < cur_win_sz - 1; ++i) {
      std::swap(_sl_pose.at(i), _sl_pose.at(i + 1));
      std::swap(_sl_P.at(i), _sl_P.at(i + 1));
      std::swap(_sl_timestamp.at(i), _sl_timestamp.at(i + 1));
    }

    _sl_timestamp.at(cur_win_sz - 1) = timestamp;
    _sl_P.at(cur_win_sz - 1) =
        P.topLeftCorner(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE);
    _sl_pose.at(cur_win_sz - 1) = latest_state;
  }
  // refresh_propagate_window(cur_win_sz, P);
  refresh_window_cross_correlation(P);

  refresh_window_landmark_cross_correlation(P, ekf_lm_pos);
  refresh_landmark_cross_correlation(P, ekf_lm_pos);
}

void SlidingWindow::initialize_landmark(
    const std::vector<std::map<SensorType, std::vector<int>>> &sw_lm_list) {
  for (size_t i = 0; i < sw_lm_list.size(); ++i) {
    for (auto it = sw_lm_list.at(i).begin(); it != sw_lm_list.at(i).end();
         ++it) {
      const auto &type = it->first;
      for (size_t j = 0; j < it->second.size(); ++j) {
        int landmark_id = it->second.at(j);

        if (!SemanticMap::GetInstance()
                 .GetLandmark(type, landmark_id)
                 ->Initialized()) {
          Eigen::MatrixXd Jx;
          SemanticMap::GetInstance().InitializeLandmark(
              type, landmark_id, _sl_timestamp.at(i), _sl_pose.at(i),
              _sl_P.at(i), Jx);
          std::cout << "Aug Landmark: " << landmark_id << std::endl;
          // SemanticMap::GetInstance().SetLandmarkCov(type, landmark_id,
          // Eigen::MatrixXd::Zero(4, 4));
          landmark_state_augmentation(type, landmark_id, i, Jx);
        }
      }
    }
  }
}

void SlidingWindow::data_check() {
  int n = this->GetCurWindowSz();
  if (n == 0) {
    return;
  }

  for (int i = 0; i < n - 1; ++i) {
    for (int j = i + 1; j < n; ++j) {
      std::pair<int, int> window_id = {i, j};
      if (!_window_cross_correlation.count(window_id)) {
        std::cout
            << "FUCKINGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG WRONG1\n";
      }
    }
  }

  std::vector<std::pair<SensorType, int>> lm_vector;
  std::map<std::pair<SensorType, int>, bool> lm_vector_flag0, lm_vector_flag1;

  for (auto it = _state_landmark.begin(); it != _state_landmark.end(); ++it) {
    for (auto itt = it->second.begin(); itt != it->second.end(); ++itt) {
      int landmark_id = *itt;
      lm_vector.push_back({it->first, landmark_id});
      lm_vector_flag0[{it->first, landmark_id}] = false;
      lm_vector_flag1[{it->first, landmark_id}] = false;
    }
  }

  if (!_state_landmark.empty() && _state_landmark.begin()->second.size() > 1) {


    for (auto it = _lm_cross_correlation.begin();
         it != _lm_cross_correlation.end(); ++it) {
      auto type0 = static_cast<SensorType>(it->first.first.type);
      auto id0 = it->first.first.id;
      auto type1 = static_cast<SensorType>(it->first.second.type);
      auto id1 = it->first.second.id;

      lm_vector_flag0.at({type0, id0}) = true;
      lm_vector_flag0.at({type1, id1}) = true;
    }

    for (auto it = lm_vector_flag0.begin(); it != lm_vector_flag0.end(); ++it) {
      if (it->second == false) {
        std::cout
            << "FUCKINGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG WRONG2\n";
        std::cout << it->first.second << std::endl;
      }
    }
  }

  for (int i = 0; i < n; ++i) {
    for (auto it = _window_lm_cross_correlation.at(i).begin();
         it != _window_lm_cross_correlation.at(i).end(); ++it) {
      auto type = static_cast<SensorType>(it->first.type);
      auto id = it->first.id;
      lm_vector_flag1.at({type, id}) = true;
    }
  }

  for (auto it = lm_vector_flag1.begin(); it != lm_vector_flag1.end(); ++it) {
    if (it->second == false) {
      std::cout
          << "FUCKINGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG WRONG3\n";
      std::cout << it->first.second << std::endl;
    }
  }


}

void SlidingWindow::landmark_state_augmentation(const SensorType &type,
                                                const int &lm_id,
                                                const int &sw_id,
                                                const Eigen::MatrixXd &Jx) {
  std::map<SensorType, std::map<int, int>> ekf_lm_pos;
  int state_sz = get_state_size(ekf_lm_pos);
  Eigen::VectorXd x;
  Eigen::MatrixXd P;
  construct_x_and_P(x, P, state_sz, ekf_lm_pos);

  // std::cout << P << std::endl;
  // std::cout << "*******************\n";

  _state_landmark[type].insert(lm_id);
  ekf_lm_pos[type].insert({lm_id, state_sz});

  Eigen::MatrixXd P_aug = P;
  int lm_state_sz;
  switch (type) {
  case SensorType::SEMANTIC_TYPE_PARKING_SLOT:
    lm_state_sz = STATE_PARKING_SLOT_SIZE;
    break;

  default:
    break;
  }

  P_aug.conservativeResize(P.rows() + lm_state_sz, P.cols() + lm_state_sz);
  int window_pos = STATE_VEHICLE_SIZE * (this->GetCurWindowSz() - sw_id - 1);
  Eigen::MatrixXd cov_window =
      P.block(window_pos, window_pos, STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE);
  P_aug.bottomRightCorner(lm_state_sz, lm_state_sz) =
      SemanticMap::GetInstance().GetLandmark(type, lm_id)->GetCov();

  P_aug.topRightCorner(P.rows(), lm_state_sz) =
      P.block(0, window_pos, P.rows(), STATE_VEHICLE_SIZE) * Jx.transpose();

  // P_aug.topRightCorner(P.rows(), lm_state_sz).setZero();

  P_aug.bottomLeftCorner(lm_state_sz, P.rows()) =
      P_aug.topRightCorner(P.rows(), lm_state_sz).transpose();

  // P_aug.topRightCorner(P.rows(), lm_state_sz).setZero();
  // P_aug.bottomLeftCorner(lm_state_sz, P.rows()).setZero();

  // std::cout << P_aug << std::endl;
  // std::cout << "##############################\n";

  // getchar();

  refresh_landmark_cross_correlation(P_aug, ekf_lm_pos);
  refresh_window_landmark_cross_correlation(P_aug, ekf_lm_pos);
  refresh_window_cross_correlation(P_aug);
}

int SlidingWindow::get_state_size(
    std::map<SensorType, std::map<int, int>> &ekf_lm_pos) {
  int state_sz = this->GetCurWindowSz() * STATE_VEHICLE_SIZE;

  for (auto it_type = _state_landmark.begin(); it_type != _state_landmark.end();
       ++it_type) {
    const auto &type = it_type->first;
    for (auto it = it_type->second.begin(); it != it_type->second.end(); ++it) {
      int landmark_id = *it;
      switch (type) {
      case SensorType::SEMANTIC_TYPE_PARKING_SLOT:
        ekf_lm_pos[type].insert({landmark_id, state_sz});
        state_sz += STATE_PARKING_SLOT_SIZE;
        break;

      default:
        break;
      }
    }
  }
  return state_sz;
}

bool SlidingWindow::AddKeyFrame(const long long ts, const Eigen::VectorXd &x,
                                const double translation_th) {
  if (_sl_pose.empty()) {
    return true;
  } else {
    int sl_sz = this->GetCurWindowSz();
    Eigen::VectorXd last_x = _sl_pose.at(sl_sz - 1);

    Eigen::Vector2d twb_cur = x.head(2);
    Eigen::Vector2d twb_last = last_x.head(2);

    double yaw_cur = x[2];
    double yaw_last = last_x[2];

    double translation_diff = (twb_cur - twb_last).norm();
    double yaw_diff = fabs(AngleDiff(yaw_cur, yaw_last));

    if (translation_diff >= translation_th ||
        yaw_diff >
            ApaParameters::GetInstance().GetEstimatorParamters().sl_angle_th) {
      return true;
    }
  }

  return false;
}

} // namespace apa_slam
