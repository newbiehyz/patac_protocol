/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/estimator.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Friday, May 9th 2025, 9:27:59 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once

#include <arpa/inet.h>
#include <unistd.h>

#include <Eigen/Eigen>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "ekf_management.h"
#include "kinematic_measurement.h"
#include "map_management.h"
#include "mea_preprocessor.h"
#include "parking_slot_tracker.h"
#include "semantic_measurement.h"
#include "tracker_base.h"

#include "sl_ekf_management.h"
#include "action_queue.h"
#include "map_io.h"

namespace apa_slam {
class EkfEstimator {
 public:
  typedef std::shared_ptr<EkfEstimator> Ptr;

  EkfEstimator();

  static EkfEstimator& GetInstance();

  void Init();

  bool GetLatestVechileState(long long& timestamp, Eigen::VectorXd& mean,
                             Eigen::MatrixXd& cov);

  void InputSemanticMea(const long long ts,
                        const std::vector<SemanticMea::Ptr>& semantic_meas);

  void InputKinematicMea(const long long ts,
                         const std::vector<KinematicMea::Ptr>& kinetic_meas);

  bool Initialized() const;

  void Reset();

  bool ProcDrPose(long long ts, const Pose& pose, long long& ts_out,
                  double& v_out, double& w_out);

 private:
  void process_odo_mea(const long long ts, const KinematicMea::Ptr odo_mea);

  void sort_semantic_meas(
      const std::vector<SemanticMea::Ptr>& semantic_meas,
      std::unordered_map<SensorType, std::vector<SemanticMea::Ptr>>&
          sorted_meas);

  void process_semantic_meas(
      const SensorType& type, const long long ts,
      const std::vector<SemanticMea::Ptr>& parking_slot_meas);

  bool get_pose(const long long ts, Pose& pose);

  double interpolate_angle(const double angle0, const double angle1,
                           const double t);
  Eigen::Vector2d interpolate_translation(const Eigen::Vector2d& twb0,
                                          const Eigen::Vector2d& twb1,
                                          const double t);

  double angle_diff(double angle1, double angle2);

  void udp();

  bool zupt();

  std::mutex _data_mutex;

  std::unordered_map<SensorType, TrackerBase::Ptr> _tracker_pools;

  std::map<long long, DrInfo> _dr_buf;

  std::vector<Pose> _dr_pose;
  std::vector<long long> _dr_timestamp;

  bool _zupt{false};
  int _zupt_sz{5};

  // socket debug
  int _socket;
  sockaddr_in _server_addr;
};
}  // namespace apa_slam
