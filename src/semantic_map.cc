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

const std::unordered_map<int, SemanticLandmark::Ptr>& SemanticMap::GetMap(
    const SensorType type) {
  std::lock_guard<std::mutex> lock(_data_mutex);
  return _map.at(type);
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
    num = _map.at(type).size();
  }

  return num;
}

void SemanticMap::ClearMap() { _map.clear(); }

}  // namespace apa_slam
