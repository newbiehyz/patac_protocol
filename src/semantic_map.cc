/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/semantic_map.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Thursday, May 8th 2025, 4:08:12 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "semantic_map.h"

namespace apa_slam {
SemanticMap::SemanticMap() {}

SemanticMap& SemanticMap::GetInstance() {
  static SemanticMap instance;
  return instance;
}

void SemanticMap::AddLandmark(const SensorType type,
                              SemanticLandmark::Ptr landmark) {
  _map[type][landmark->GetId()] = landmark;
}

bool SemanticMap::HasLandmark(const SensorType type, const int id) {
  if (!_map.count(type)) {
    return false;
  }

  if (!_map.at(type).count(id)) {
    return false;
  }

  return true;
}

bool SemanticMap::HasMap(const SensorType type) const {
  return _map.count(type);
}

const std::map<int, SemanticLandmark::Ptr> SemanticMap::GetMap(
    const SensorType type) {
  std::lock_guard<std::mutex> lock(_data_mutex);
  return _map.at(type);
}

void SemanticMap::InitializeLandmark(const SensorType type, const int id,
                                     const Eigen::VectorXd& vehicle_mean,
                                     Eigen::MatrixXd& vehicle_P,
                                     Eigen::MatrixXd& Jx) {
  _map.at(type).at(id)->InitializeLandmark(vehicle_mean, vehicle_P, Jx);
}

void SemanticMap::InitializeLandmark(const SensorType type, const int id,
                                     const long long timestamp,
                                     const Eigen::VectorXd& vehicle_mean,
                                     Eigen::MatrixXd& vehicle_P,
                                     Eigen::MatrixXd& Jx) {
  _map.at(type).at(id)->InitializeLandmark(timestamp, vehicle_mean, vehicle_P,
                                           Jx);
}
Eigen::MatrixXd SemanticMap::GetLandmarkCov(const SensorType& type,
                                            const int id) {
  return _map.at(type).at(id)->GetCov();
}

void SemanticMap::SetLandmarkCov(const SensorType& type, const int id,
                                 const Eigen::MatrixXd& cov) {
  _map.at(type).at(id)->SetCov(cov);
}

void SemanticMap::SetLandmarkMean(const SensorType& type, const int id,
                                  const Eigen::VectorXd& mean) {
  _map.at(type).at(id)->SetMean(mean);
}

void SemanticMap::TagMarginalization(const long long timestamp) {
  for (auto it_type = _map.begin(); it_type != _map.end(); ++it_type) {
    const auto& type = it_type->first;
    for (auto it_lm = it_type->second.begin(); it_lm != it_type->second.end();
         ++it_lm) {
      int lm_id = it_lm->first;
      it_lm->second->TagMarginalization(timestamp);
    }
  }
}

void SemanticMap::SetTargetSlotId(const int id) {
  if (!HasLandmark(SEMANTIC_TYPE_PARKING_SLOT, id)) {
    return;
  }

  SemanticLandmark::Ptr lm = _map.at(SEMANTIC_TYPE_PARKING_SLOT).at(id);
  ParkingSlotLandmark::Ptr ps_lm =
      std::dynamic_pointer_cast<ParkingSlotLandmark>(lm);
  ps_lm->SetTarget();
}

bool SemanticMap::GetFullLocalMap(
    const SensorType& type, const Pose& pose,
    std::vector<SemanticLandmark::Ptr>& local_map) {
  if (!SemanticMap::GetInstance().HasMap(type)) {
    return false;
  }
  double range;
  switch (type) {
    case SensorType::SEMANTIC_TYPE_PARKING_SLOT:
      range = ApaParameters::GetInstance()
                  .GetEstimatorParamters()
                  .slot_local_map_range;
      break;

    default:
      break;
  }
  // std::cout << "Pose: " << pose.x << " " << pose.y << " " << pose.yaw <<
  // std::endl;
  for (auto it_lm = _map.at(type).begin(); it_lm != _map.at(type).end();
       ++it_lm) {
    Eigen::MatrixXd lm_data = it_lm->second->GetLandmarkData();
    Eigen::Vector2d corner =
        (lm_data.col(0).head(2) + lm_data.col(1).head(2)) * .5f;
    double distance = (Eigen::Vector2d(pose.x, pose.y) - corner).norm();
    if (distance < range) {
      local_map.push_back(it_lm->second);
      // std::cout << "local map id: " << it_lm->first << " |||| \n"
      //           <<
      //           it_lm->second->GetLandmarkData().col(0).head(2).transpose()
      //           << std::endl
      //           <<
      //           it_lm->second->GetLandmarkData().col(1).head(2).transpose()
      //           << std::endl;
    }
  }

  return true;
}

void SemanticMap::MarginLandmark(const SensorType& type, const int& id) {
  _map.at(type).erase(id);
}

const SemanticLandmark::Ptr SemanticMap::GetLandmark(const SensorType& type,
                                                     const int& id) {
  return _map.at(type).at(id);
}

void SemanticMap::GetSlidingWindowEKFDataList(
    const std::vector<long long>& slw_timestamp,
    std::vector<std::map<SensorType, std::vector<int>>>& sw_lm_list,
    std::map<SensorType, std::set<int>>& marginalization_list) {
  sw_lm_list.resize(slw_timestamp.size());
  for (auto it_type = _map.begin(); it_type != _map.end(); ++it_type) {
    const auto& type = it_type->first;
    switch (type) {
      case SensorType::SEMANTIC_TYPE_PARKING_SLOT: {
        for (auto it_lm = it_type->second.begin();
             it_lm != it_type->second.end(); ++it_lm) {
          // it_lm->second->EraseMeasPre(slw_timestamp.at(0));

          std::vector<int> ob_window_id;
          int sl_ob_num = it_lm->second->GetSlidingWindowObservationTimes(
              slw_timestamp, ob_window_id);
          if (sl_ob_num > ApaParameters::GetInstance()
                              .GetEstimatorParamters()
                              .slot_min_tracking_times) {
            for (int i = 0; i < ob_window_id.size(); ++i) {
              sw_lm_list.at(ob_window_id.at(i))[type].push_back(it_lm->first);
            }
          }
          ParkingSlotLandmark::Ptr ps = std::dynamic_pointer_cast<ParkingSlotLandmark>(it_lm->second);

          if (sl_ob_num == 0 && it_lm->second->NeedMargin(slw_timestamp) && !ps->IsTarget()) {
            marginalization_list[type].insert(it_lm->first);
          }
        }
      } break;

      default:
        break;
    }
  }
}

void SemanticMap::GetEKFDataList(
    std::map<SensorType, std::set<int>>& augmentation_list,
    std::map<SensorType, std::set<int>>& update_list,
    std::map<SensorType, std::set<int>>& marginalization_list) {
  for (auto it_type = _map.begin(); it_type != _map.end(); ++it_type) {
    const auto& type = it_type->first;
    for (auto it_lm = it_type->second.begin(); it_lm != it_type->second.end();
         ++it_lm) {
      int lm_id = it_lm->first;
      if (it_lm->second->NeedInitialize()) {
        augmentation_list[type].insert(lm_id);
      }

      if (it_lm->second->Initialized()) {
        if (it_lm->second->NeedUpdate()) {
          update_list[type].insert(lm_id);
          it_lm->second->SetUpdateFlag(false);
        }

        if (it_lm->second->NeedMargin()) {
          marginalization_list[type].insert(lm_id);
        }
      }
    }
  }
}

void SemanticMap::AddMea(const SensorType type, const int id,
                         const SemanticMea::Ptr mea, const Pose& mea_pose) {
  if (!_map.count(type)) {
    std::cout << "No such Map\n";
    return;
  }

  if (!_map.at(type).count(id)) {
    std::cout << "No such Id\n";
    return;
  }
  double timestamp = mea->GetMeaTimestamp();
  _map.at(type).at(id)->AddSemanticMea(timestamp, mea_pose, mea);
}

int SemanticMap::GetMapLandmarkNum(const SensorType& type) {
  int num = 0;
  if (!_map.count(type)) {
    num = 0;
  } else {
    num = _map.at(type).rbegin()->first + 1;
  }

  return num;
}

int SemanticMap::GetMapInitializedLandmarkNum(const SensorType& type) {
  int num = 0;
  if (!_map.count(type)) {
    num = 0;
  } else {
    for (auto it = _map.at(type).begin(); it != _map.at(type).end(); ++it) {
      if (it->second->Initialized()) {
        ++num;
      }
    }
  }

  return num;
}

void SemanticMap::ClearMap() {
  std::lock_guard<std::mutex> lock(_data_mutex);

  _map.clear();
}

}  // namespace apa_slam
