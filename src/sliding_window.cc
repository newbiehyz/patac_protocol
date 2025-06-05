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

int SlidingWindow::GetCurWindowSz() { return _sl_timestamp.size(); }

bool SlidingWindow::GetLatestTimestamp(long long &timestmap) {
  if (_sl_timestamp.empty()) {
    return false;
  }

  timestmap = _sl_timestamp.back();
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

  ConstructxAndP(x, P, lm_pos);

  Eigen::VectorXd v_state = x.head(3);
}

void SlidingWindow::ConstructxAndP(
    Eigen::VectorXd &x, Eigen::MatrixXd &P,
    std::map<SensorType, std::map<int, int>> &ekf_lm_pos) {
  int n_state = this->get_state_size(ekf_lm_pos);

  P = Eigen::MatrixXd::Zero(n_state, n_state);
  x = Eigen::VectorXd::Zero(n_state);

  for (int i = 0; i < this->GetCurWindowSz(); ++i) {
    int start_id = STATE_VEHICLE_SIZE * (this->GetCurWindowSz() - i - 1);
    x.segment(start_id, STATE_VEHICLE_SIZE) = _sl_pose[i];
    P.block(start_id, start_id, STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) = _sl_P[i];
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

void SlidingWindow::ConstructEKF(
    Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::VectorXd &residual,
    Eigen::MatrixXd &H, Eigen::MatrixXd &R,
    std::map<SensorType, std::map<int, int>> &ekf_lm_pos) {
  int n_state = this->get_state_size(ekf_lm_pos);
}

SlidingWindow &SlidingWindow::GetInstance() {
  static SlidingWindow instance;
  return instance;
}

int SlidingWindow::get_state_size(
    std::map<SensorType, std::map<int, int>> &ekf_lm_pos) {}

bool SlidingWindow::AddKeyFrame(const long long ts, const Eigen::VectorXd &x) {
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

    if (translation_diff > ApaParameters::GetInstance()
                               .GetEstimatorParamters()
                               .sl_translation_th ||
        yaw_diff >
            ApaParameters::GetInstance().GetEstimatorParamters().sl_angle_th) {
      return true;
    }
  }

  return false;
}

} // namespace apa_slam
