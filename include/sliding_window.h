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
#include <map>
#include <memory>
#include <mutex>

#include "apa_parameters.h"
#include "cross_correlation_id.h"
#include "math_utils.h"
#include "semantic_map.h"

using WindowCrossCorrelation =
    std::map<std::pair<int, int>,
             Eigen::MatrixXd>;  // std::pair<id0, id1> P01, id0 < id1,
                                // id0=older, id1=newer
using WindowLMCrossCorrelation = std::vector<
    std::unordered_map<apa_slam::CrossCorrelationId, Eigen::MatrixXd>>;
using LMCrossCorrelation =
    std::unordered_map<apa_slam::CrossCorrelationKey, Eigen::MatrixXd>;
namespace apa_slam {
class SlidingWindow {
 public:
  typedef std::shared_ptr<SlidingWindow> Ptr;
  SlidingWindow();
  static SlidingWindow &GetInstance();
  void Update(const long long timestamp);
  void Init();
  bool Initialized();
  int GetCurWindowSz();
  bool GetSlidingWindowStatus(const int id, long long &timestamp,
                              Eigen::VectorXd &x, Eigen::MatrixXd &P);
  void InitializeSlw(const long long ts, const Eigen::VectorXd &x,
                     const Eigen::MatrixXd &P);

  void Propagate(
      const long long ts,
      const std::vector<std::pair<long long, Eigen::VectorXd>> &odo_meas);
  void ConstructEKF(Eigen::VectorXd &x, Eigen::MatrixXd &P,
                    Eigen::VectorXd &residual, Eigen::MatrixXd &H,
                    Eigen::MatrixXd &R,
                    std::map<SensorType, std::map<int, int>> &ekf_lm_pos);

  long long GetSlwTimestamp(const int id);

  bool AddKeyFrame(const long long ts, const Eigen::VectorXd &x);

 private:
  void landmark_state_augmentation(const SensorType &type, const int &lm_id,
                                   const int &sw_id, const Eigen::MatrixXd &Jx);

  void construct_x_and_P(
      Eigen::VectorXd &x, Eigen::MatrixXd &P, const int &state_sz,
      const std::map<SensorType, std::map<int, int>> &ekf_lm_pos);

  int get_state_size(
      std::map<SensorType, std::map<int, int>>
          &ekf_lm_pos);  // std::map<int, int> ===== <landmarkId, position>

  void initialize_landmark(
      const std::vector<std::map<SensorType, std::vector<int>>> &sw_lm_list);

  void update_propagate_window_status(
      const long long &timestamp, const Eigen::VectorXd &latest_state,
      const Eigen::MatrixXd &P_aug,
      const std::map<SensorType, std::map<int, int>> &ekf_lm_pos);

  void set_landmark_cross_correlation(const CrossCorrelationId &id0,
                                      const CrossCorrelationId &id1,
                                      const Eigen::MatrixXd &correlation);

  Eigen::MatrixXd get_landmark_cross_correlation(const CrossCorrelationId &id0,
                                                 const CrossCorrelationId &id1);

  CrossCorrelationKey make_landmark_cross_correlation_key(
      const CrossCorrelationId &id0, const CrossCorrelationId &id1);

  Eigen::MatrixXd get_window_cross_correlation(const int id0, const int id1);

  void set_window_cross_correlation(const int id0, const int id1,
                                    const Eigen::MatrixXd &correlation);

  std::pair<int, int> make_window_correlation_key(const int id0, const int id1);

  void set_window_landmark_cross_correlation(
      const int &window_id, const SensorType &type, const int &lm_id,
      const Eigen::MatrixXd &correlation);

  Eigen::MatrixXd get_window_landmark_cross_correlation(const int &window_id,
                                                        const SensorType &type,
                                                        const int &lm_id);

  void refresh_propagate_window(const int win_sz, const Eigen::MatrixXd &P);

  void refresh_landmark(
      const Eigen::MatrixXd &P,
      const std::map<SensorType, std::map<int, int>> &ekf_lm_pos);

  void refresh_propagate_window_landmark(
      const Eigen::MatrixXd &P,
      const std::map<SensorType, std::map<int, int>> &ekf_lm_pos);

  void refresh_update_window(const Eigen::MatrixXd &P);

  void refresh_update_window_landmark(
      const Eigen::MatrixXd &P,
      const std::map<SensorType, std::map<int, int>> &ekf_lm_pos);

  bool _initialized{false};

  std::vector<long long> _sl_timestamp;
  std::vector<Eigen::VectorXd> _sl_pose;
  std::vector<Eigen::MatrixXd> _sl_P;

  Eigen::MatrixXd _N;  // odo measurement

  WindowCrossCorrelation _window_cross_correlation;
  WindowLMCrossCorrelation _window_lm_cross_correlation;
  LMCrossCorrelation _lm_cross_correlation;

  std::map<SensorType, std::set<int>> _state_landmark;
};
}  // namespace apa_slam
