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

namespace apa_slam {
class SlEKFManagement {
public:
  typedef std::shared_ptr<SlEKFManagement> Ptr;
  SlEKFManagement();
  void Init();
  static SlEKFManagement &GetInstance();
  void Propagate(const long long timestamp, const double v, const double w);
  void Update(const long long timestamp);
  bool Initialized();
  bool GetLatestVechileState(long long &timestamp, Eigen::VectorXd &mean,
                             Eigen::MatrixXd &cov);

private:
  void erase_pres(const long long timestamp);

  Eigen::VectorXd _vehicle_x;
  double _vehicle_v;
  double _vehicle_w;
  long long _ts;
  bool _initialized;

  std::map<long long, Eigen::VectorXd> _odo_meas;
  std::map<long long, std::pair<Eigen::VectorXd, Eigen::MatrixXd>> _pre_states;
};

} // namespace apa_slam
