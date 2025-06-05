/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/sliding_window.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Friday, May 30th 2025, 5:09:23 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once
#include <Eigen/Eigen>
#include <deque>
#include <memory>
#include <map>
#include "apa_parameters.h"
#include "cross_correlation_id.h"
#include "math_utils.h"

using WindowCrossCorrelation = std::map<std::pair<int, int>, Eigen::MatrixXd>;
using WindowLMCrossCorrelation = std::vector<std::unordered_map<apa_slam::CrossCorrelationId, Eigen::MatrixXd>>;
using LMCrossCorrlation = std::unordered_map<apa_slam::CrossCorrelationKey, Eigen::MatrixXd>;
namespace apa_slam {
class SlidingWindow {
public:
  typedef std::shared_ptr<SlidingWindow> Ptr;
  SlidingWindow();
  static SlidingWindow &GetInstance();
  void Init();
  bool Initialized();
  int GetCurWindowSz();
  bool GetLatestTimestamp(long long &timestmap);
  void InitializeSlw(const long long ts, const Eigen::VectorXd &x,
                     const Eigen::MatrixXd &P);
  void
  Propagate(const long long ts,
            const std::vector<std::pair<long long, Eigen::VectorXd>> &odo_meas);
  void ConstructEKF(Eigen::VectorXd &x, Eigen::MatrixXd &P,
                    Eigen::VectorXd &residual, Eigen::MatrixXd &H,
                    Eigen::MatrixXd &R,
                    std::map<SensorType, std::map<int, int>> &ekf_lm_pos);
  void ConstructxAndP(Eigen::VectorXd &x, Eigen::MatrixXd &P,
                      std::map<SensorType, std::map<int, int>> &ekf_lm_pos);
  long long GetSlwTimestamp(const int id);

  bool AddKeyFrame(const long long ts, const Eigen::VectorXd &x);

private:
  int get_state_size(std::map<SensorType, std::map<int, int>> &ekf_lm_pos);

  bool _initialized{false};

  std::vector<long long> _sl_timestamp;
  std::vector<Eigen::VectorXd> _sl_pose;
  std::vector<Eigen::MatrixXd> _sl_P;

  Eigen::MatrixXd _N; // odo measurement

  WindowCrossCorrelation _window_cross_correlation;
  WindowLMCrossCorrelation _window_lm_cross_correlation;
  LMCrossCorrlation _lm_cross_correlation;

};
} // namespace apa_slam
