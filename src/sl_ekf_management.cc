/*
 * Filename:
 * /home/yukan/Documents/apa_local_mapping/src/sl_ekf_management.cc
 * Path: /home/yukan/Documents/apa_local_mapping/src/
 * Created Date: Saturday, May 31st 2025, 4:29:57 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include "sl_ekf_management.h"

namespace apa_slam {
SlEKFManagement::SlEKFManagement() {

  _vehicle_x = Eigen::VectorXd::Zero(3);
  _initialized = false;
}

void SlEKFManagement::Init() { SlidingWindow::GetInstance().Init(); }

SlEKFManagement &SlEKFManagement::GetInstance() {
  static SlEKFManagement instance;
  return instance;
}

bool SlEKFManagement::GetLatestVechileState(long long &timestamp,
                                            Eigen::VectorXd &mean,
                                            Eigen::MatrixXd &cov) {
  return true;
}

void SlEKFManagement::Propagate(const long long timestamp, const double v,
                                const double w) {
  _vehicle_w = w;
  _vehicle_v = v;
  _ts = timestamp;

  if (!_initialized) {
    _initialized = true;

    return;
  }

  double timescale =
      ApaParameters::GetInstance().GetEstimatorParamters().time_scale;
  double dt = static_cast<double>(timestamp - _ts) * timescale;

  Eigen::Vector2d twb = _vehicle_x.head(2);
  double yaw = _vehicle_x[2];
  Eigen::Rotation2Dd rot(yaw);
  Eigen::Matrix2d Rwb = rot.toRotationMatrix();
  Eigen::Vector2d dir = Rwb.col(0);

  Eigen::MatrixXd P;
  Eigen::VectorXd x;

  double noise_v =
      ApaParameters::GetInstance().GetEstimatorParamters().odo_velocity_noise;
  double noise_w = ApaParameters::GetInstance()
                       .GetEstimatorParamters()
                       .odo_angular_velocity_noise;

  Eigen::MatrixXd N = Eigen::MatrixXd::Zero(NOISE_ODO_SIZE, NOISE_ODO_SIZE);
  N.diagonal()[0] = noise_v * noise_v;
  N.diagonal()[1] = noise_w * noise_w;

  Eigen::MatrixXd Fx = Eigen::MatrixXd::Identity(3, 3);
  Eigen::Vector2d d_dir_yaw(-std::sin(yaw), std::cos(yaw));
  Fx.topRightCorner(2, 1) = d_dir_yaw * v * dt;

  Eigen::MatrixXd Fn = Eigen::MatrixXd::Zero(3, 2);
  Fn.topLeftCorner(2, 1) = dir * dt;
  Fn(2, 1) = dt;
  Eigen::MatrixXd P_last;
  if (_pre_states.size() == 0) {
    P_last = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  } else {
    P_last = _pre_states.rbegin()->second.second;
  }

  P = Fx * P_last * Fx.transpose() + Fn * N * Fn.transpose();
  x = Eigen::VectorXd::Zero(STATE_VEHICLE_SIZE);

  twb += dir * v * dt;
  yaw += dt * w;

  x.head(2) = twb;
  x[2] = yaw;

  _pre_states[timestamp] = {x, P};

  _vehicle_x = x;
  _ts = timestamp;

  // hist information
  Eigen::VectorXd odo_mea = Eigen::VectorXd::Zero(2);
  odo_mea[0] = v;
  odo_mea[1] = w;
  _odo_meas[timestamp] = odo_mea;

  if (SlidingWindow::GetInstance().Initialized()) {
    int sl_sz = SlidingWindow::GetInstance().GetCurWindowSz();
    long long sl_timestmap =
        SlidingWindow::GetInstance().GetSlwTimestamp(sl_sz - 1);
    erase_pres(sl_timestmap);
  }
}

void SlEKFManagement::erase_pres(const long long timestamp) {
  auto it = _odo_meas.begin();
  while (it != _odo_meas.end() && it->first < timestamp) {
    it = _odo_meas.erase(it);
  }

  auto it_state = _pre_states.begin();
  while (it_state != _pre_states.end() && it_state->first < timestamp) {
    it_state = _pre_states.erase(it_state);
  }
}

void SlEKFManagement::Update(const long long timestamp) {

  if (!SlidingWindow::GetInstance().Initialized()) {
    if (timestamp < _pre_states.begin()->first ||
        timestamp > _pre_states.rbegin()->first) {
      return;
    }
    auto it = _pre_states.lower_bound(timestamp);
    Eigen::VectorXd x = it->second.first;
    Eigen::MatrixXd P = it->second.second;
    SlidingWindow::GetInstance().InitializeSlw(timestamp, x, P);
  } else {
    int slw_sz = SlidingWindow::GetInstance().GetCurWindowSz();
    long long last_slw_timestamp =
        SlidingWindow::GetInstance().GetSlwTimestamp(slw_sz - 1);
    auto it_odo = _odo_meas.lower_bound(last_slw_timestamp);
    std::vector<std::pair<long long, Eigen::VectorXd>> odo_for_update;
    while (it_odo != _odo_meas.end() && it_odo->first <= timestamp) {
      odo_for_update.push_back({it_odo->first, it_odo->second});
      ++it_odo;
    }

    auto it_state = _pre_states.lower_bound(timestamp);
    if (SlidingWindow::GetInstance().AddKeyFrame(timestamp,
                                                 it_state->second.first)) {
      SlidingWindow::GetInstance().Propagate(timestamp, odo_for_update);
    }
  }
}

} // namespace apa_slam
