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

bool SemanticMap::HasMap(const SensorType type) { return _map.count(type); }

const std::unordered_map<int, SemanticLandmark::Ptr>& SemanticMap::GetMap(
    const SensorType type) {
  return _map.at(type);
}

void SemanticMap::ProcessMatching(const std::vector<SemanticMea::Ptr>& meas,
                                  const std::vector<int>& matching_res,
                                  const Pose& mea_pose) {
  const auto mea_type = meas.at(0)->GetSemanticMeaType();
  if (mea_type == SEMANTIC_TYPE_PARKING_SLOT) {
    process_slot_matching(meas, matching_res, mea_pose);
  }
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

void SemanticMap::process_slot_matching(
    const std::vector<SemanticMea::Ptr>& meas,
    const std::vector<int>& matching_res, const Pose& mea_pose) {
  for (size_t i = 0; i < matching_res.size(); ++i) {
    if (matching_res.at(i) == -1) {
      Eigen::MatrixXd lm_data =
          Eigen::MatrixXd::Zero(PARKING_SLOT_DATA_ROWS, PARKING_SLOT_DATA_COLS);
      int id = this->GetMapLandmarkNum(SEMANTIC_TYPE_PARKING_SLOT) + 1;
      SemanticLandmark::Ptr slot_lm = std::make_shared<ParkingSlotLandmark>(id, lm_data.data());
    }
  }
}

}  // namespace apa_slam
