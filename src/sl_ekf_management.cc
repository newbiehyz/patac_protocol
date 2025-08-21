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
SlEKFManagement::SlEKFManagement() {}

void SlEKFManagement::Reset() {
  _initialized = false;
  _odo_meas.clear();
  _pre_states.clear();
  // _cached_slot_ids.clear();
  this->Init();
}

void SlEKFManagement::Init() {
  _vehicle_x = Eigen::VectorXd::Zero(STATE_VEHICLE_SIZE);
  _vehicle_P =
      Eigen::MatrixXd::Identity(STATE_VEHICLE_SIZE, STATE_VEHICLE_SIZE) *
      0.001f;
  _initialized = false;
  SlidingWindow::GetInstance().Init();
}

SlEKFManagement &SlEKFManagement::GetInstance() {
  static SlEKFManagement instance;
  return instance;
}

bool SlEKFManagement::GetLatestVechileState(long long &timestamp,
                                            Eigen::VectorXd &mean,
                                            Eigen::MatrixXd &cov) {
  if (SlidingWindow::GetInstance().Initialized() == false) {
    return false;
  }
  std::lock_guard<std::mutex> lock(_data_mutex);

  timestamp = _ts;
  mean = _vehicle_x;
  cov = _vehicle_P;
  return true;
}

bool SlEKFManagement::Initialized() { return _initialized; }

void SlEKFManagement::Propagate(const long long timestamp, const double v,
                                const double w) {
  _vehicle_w = w;
  _vehicle_v = v;

  if (!_initialized) {
    _initialized = true;
    _ts = timestamp;
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

  P = Fx * _vehicle_P * Fx.transpose() + Fn * N * Fn.transpose();
  x = Eigen::VectorXd::Zero(STATE_VEHICLE_SIZE);

  twb += dir * v * dt;
  yaw += dt * w;

  x.head(2) = twb;
  x[2] = yaw;

  _pre_states[timestamp] = {x, P};

  _vehicle_x = x;
  _vehicle_P = P;
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
    // erase_pres(sl_timestmap);
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

void SlEKFManagement::Update(const long long timestamp, bool zupt) {
  // std::lock_guard<std::mutex> lock(_data_mutex);
  if (_pre_states.empty()) {
    return;
  }
  if (timestamp < _pre_states.begin()->first ||
      timestamp > _pre_states.rbegin()->first) {
    // if (timestamp < _pre_states.begin()->first) {
    //   std::cout << "FATAL ERROR: UPDATE TIMESTAMP BEFORE odo_buffer
    //   begins()\n";
    // }

    if (timestamp > _pre_states.rbegin()->first) {
      // std::cout << "FATAL ERROR: UPDATE TIMESTAMP AFTER odo_buffer
      // rbegins()\n";
      std::cout << "\033[1;33mFATAL ERROR: UPDATE TIMESTAMP AFTER odo_buffer "
                   "Rbegins\033[0m"
                << std::endl;
    }
    return;
  }
  if (!SlidingWindow::GetInstance().Initialized()) {
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
    double v = it_odo->second.x();
    double w = it_odo->second.y();
    while (it_odo != _odo_meas.end() && it_odo->first <= timestamp) {
      odo_for_update.push_back({it_odo->first, it_odo->second});
      ++it_odo;
    }

    auto it_state = _pre_states.lower_bound(timestamp);
    double translation_th =
        ApaParameters::GetInstance().GetEstimatorParamters().sl_translation_th;
    // if (zupt) {
    //   translation_th = .0f;
    // }

    if (fabs(v) < 1e-4 && !only_localization &&
        SlidingWindow::GetInstance().GetCurWindowSz() <
            ApaParameters::GetInstance().GetEstimatorParamters().window_size) {
      translation_th = .0f;
    }

    if (only_localization) {
      translation_th *= 0.5;
    }

    if (SlidingWindow::GetInstance().AddKeyFrame(
            timestamp, it_state->second.first, translation_th)) {
      SlidingWindow::GetInstance().Propagate(timestamp, odo_for_update);
      Eigen::VectorXd x, residual;
      Eigen::MatrixXd P, R, Hx;
      std::map<SensorType, std::map<int, int>> ekf_lm_pos;
      std::map<SensorType, std::set<int>> marginalization_list;
      SlidingWindow::GetInstance().ConstructEKF(
          x, P, residual, Hx, R, ekf_lm_pos, marginalization_list);
      if (vis_meas.startMapping && !vis_meas.startLocalization)
        save_slot_map_cache(timestamp);


      if (residual.size() != 0) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(
            Hx, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd U = svd.matrixU();  // m x n
        Eigen::VectorXd residual_compressed = U.transpose() * residual;
        Eigen::MatrixXd H_compressed =
            svd.singularValues().asDiagonal() * svd.matrixV().transpose();
        Eigen::MatrixXd R_compressed = U.transpose() * R * U;

        Eigen::MatrixXd S =
            H_compressed * P * H_compressed.transpose() + R_compressed;
        S.diagonal() += 1e-7 * Eigen::VectorXd::Ones(S.diagonal().size());
        Eigen::MatrixXd Sinv = S.inverse();
        Eigen::MatrixXd K = P * H_compressed.transpose() * Sinv;
        x = x + K * residual_compressed;
        P = P -
            K * (H_compressed * P * H_compressed.transpose() + R_compressed) *
                K.transpose();
        // Eigen::MatrixXd S = Hx * P * Hx.transpose() + R;
        // Eigen::MatrixXd Sinv = S.inverse();
        // Eigen::MatrixXd K = P * Hx.transpose() * Sinv;
        // x = x + K * residual;
        // P = P - K * (Hx * P * Hx.transpose() + R) * K.transpose();

        P = (P + P.transpose()) * 0.5;
        SlidingWindow::GetInstance().UpdateEKF(x, P, ekf_lm_pos,
                                               marginalization_list);
        // SlidingWindow::GetInstance().ConstructEKF(
        //     x, P, residual, Hx, R, ekf_lm_pos, marginalization_list);

        // getchar();
      }
    }
  }

  long long latest_sl_timestamp;
  Eigen::VectorXd latest_sl_x;
  Eigen::MatrixXd latest_sl_P;

  if (SlidingWindow::GetInstance().Initialized()) {
    int sl_sz = SlidingWindow::GetInstance().GetCurWindowSz();
    if (SlidingWindow::GetInstance().GetSlidingWindowStatus(
            sl_sz - 1, latest_sl_timestamp, latest_sl_x, latest_sl_P)) {
      auto it_odo = _odo_meas.lower_bound(latest_sl_timestamp);
      _vehicle_w = it_odo->second.y();
      _vehicle_v = it_odo->second.x();
      {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _ts = latest_sl_timestamp;
        _vehicle_P = latest_sl_P;
        _vehicle_x = latest_sl_x;
      }
      while (it_odo != _odo_meas.end()) {
        this->Propagate(it_odo->first, it_odo->second.x(), it_odo->second.y());
        ++it_odo;
      }
    }
  }
}



void SlEKFManagement::save_slot_map_cache(const long long ts) {
  if (!SemanticMap::GetInstance().HasMap(SEMANTIC_TYPE_PARKING_SLOT)) {
    return;
  }

  const auto& slot_map = SemanticMap::GetInstance().GetMap(SEMANTIC_TYPE_PARKING_SLOT);
  
  patac_hpp::ParkingSlotList slot_map_list;
  slot_map_list.set_timestamp(ts);
  uint32_t slot_map_count = 0;

  for (const auto& slot_pair : slot_map) {
    int slot_id = slot_pair.first;
    const auto& slot_landmark = slot_pair.second;

    // for finding already cached
    if (_cached_slot_ids.find(slot_id) != _cached_slot_ids.end()) {
      continue;
    }
    
    if (!slot_landmark->Initialized()) {
      continue;
    }
    
    auto slot = std::dynamic_pointer_cast<ParkingSlotLandmark>(slot_landmark);
    if (!slot) {
      continue;
    }

    Eigen::MatrixXd slot_data = slot->GetLandmarkData();
    Eigen::MatrixXd slot_cov = slot->GetCov();
    
    patac_hpp::ParkingSlot* parking_slot = slot_map_list.add_parking_slot_list();
    parking_slot->set_id(slot_id);
    parking_slot->set_valid(1);

    auto attr = slot->GetAttribute();
    switch (attr.slot_type) {
      case Vertical:
        parking_slot->set_type(patac_hpp::SlotTypeVertical);
        break;
      case Horizontal:
        parking_slot->set_type(patac_hpp::SlotTypeParallel);
        break;
      case Oblique:
        parking_slot->set_type(patac_hpp::SlotTypeOblique);
        break;
      default:
        parking_slot->set_type(patac_hpp::SlotTypeUnknown);
        break;
    }
    
    if (attr.parkable) {
      parking_slot->set_occupancy(patac_hpp::OccupancyStatusNotOccupied);
    } else {
      parking_slot->set_occupancy(patac_hpp::OccupancyStatusOccupied);
    }
    
    patac_hpp::Point2f* point0 = parking_slot->add_points();
    point0->set_x(slot_data(0,0));
    point0->set_y(slot_data(1,0));
    patac_hpp::Point2f* point1 = parking_slot->add_points();
    point1->set_x(slot_data(0,1));
    point1->set_y(slot_data(1,1));
    
    parking_slot->set_source(patac_hpp::ParkingSourceIpm);
     
    _cached_slot_ids.insert(slot_id); // for finding already cached
    slot_map_count++;
  }
  
  slot_map_list.set_num_parking_slot(slot_map_count);

  if (slot_map_count > 0) {
    std::lock_guard<std::mutex> lock(_slot_map_data_mutex);
    _slot_map_data_cache.push_back(slot_map_list);
  }
}




std::vector<patac_hpp::ParkingSlotList> SlEKFManagement::GetCachedSlotMapData() {
  std::lock_guard<std::mutex> lock(_slot_map_data_mutex);
  return _slot_map_data_cache;
}

void SlEKFManagement::ClearSlotMapDataCache() {
  std::lock_guard<std::mutex> lock(_slot_map_data_mutex);
  _slot_map_data_cache.clear();
}




}  // namespace apa_slam
