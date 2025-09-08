/*
 * Filename: /home/yukan/Documents/apa_local_mapping/include/sl_ekf_management.h
 * Path: /home/yukan/Documents/apa_local_mapping/include
 * Created Date: Saturday, May 31st 2025, 4:33:29 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#pragma once

#include <memory>
#include <set>
#include <unordered_map>

#include <Eigen/Eigen>

#include "apa_parameters.h"
#include "sliding_window.h"

#include "patac_slot.pb.h"

namespace apa_slam {
class SlEKFManagement {
public:
  typedef std::shared_ptr<SlEKFManagement> Ptr;
  SlEKFManagement();
  void Init();
  static SlEKFManagement &GetInstance();
  void Propagate(const long long timestamp, const double v, const double w);
  void Update(const long long timestamp, bool zupt);
  bool Initialized();
  bool GetLatestVechileState(long long &timestamp, Eigen::VectorXd &mean,
                             Eigen::MatrixXd &cov);
  void Reset();

  //protobuf cache
  std::vector<patac_hpp::ParkingSlotList> GetCachedSlotMapData();
  void ClearSlotMapDataCache();
  patac_hpp::Trajectory GetCompleteTrajectory() const;

private:
  void erase_pres(const long long timestamp);

  Eigen::VectorXd _vehicle_x;
  Eigen::MatrixXd _vehicle_P;
  double _vehicle_v;
  double _vehicle_w;
  long long _ts;
  bool _initialized;

  std::map<long long, Eigen::VectorXd> _odo_meas;
  std::map<long long, std::pair<Eigen::VectorXd, Eigen::MatrixXd>> _pre_states;

  std::mutex _data_mutex;

  //protobuf cache
  void save_slot_map_cache(const long long ts);
  std::vector<patac_hpp::ParkingSlotList> _slot_map_data_cache;
  std::mutex _slot_map_data_mutex;

  std::set<int> _cached_slot_ids; // for finding already cached
  std::map<long long, patac_hpp::TrajectoryPoint> _complete_trajectory;
  mutable std::mutex _complete_trajectory_mutex;
  int _complete_trajectory_id_counter = 0;

};

} // namespace apa_slam
