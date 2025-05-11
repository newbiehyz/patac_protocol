/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/map_manager.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Saturday, May 10th 2025, 4:12:33 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "map_management.h"

namespace apa_slam {
MapManagement::MapManagement() {}

MapManagement& MapManagement::GetInstance() {
  static MapManagement instance;
  return instance;
}

void MapManagement::ProcessMatching(const std::vector<SemanticMea::Ptr>& meas,
                                    const std::vector<int>& map_matching,
                                    const Pose& mea_pose,
                                    const SensorType& type) {
  switch (type) {
    case SEMANTIC_TYPE_PARKING_SLOT:
      process_slot_matching(meas, map_matching, mea_pose);
      break;

    default:
      break;
  }
}

void MapManagement::process_slot_matching(
    const std::vector<SemanticMea::Ptr>& meas,
    const std::vector<int>& map_matching, const Pose& mea_pose) {
  for (size_t i = 0; i < map_matching.size(); ++i) {
    int map_id = map_matching.at(i);

    if (SemanticMap::GetInstance().HasLandmark(SEMANTIC_TYPE_PARKING_SLOT,
                                               map_id)) {
    } else {
      Eigen::MatrixXd lm_data =
          Eigen::MatrixXd::Zero(PARKING_SLOT_DATA_ROWS, PARKING_SLOT_DATA_COLS);

      Eigen::Vector2d twb(mea_pose.x, mea_pose.y);
      Eigen::Rotation2Dd rot(mea_pose.yaw);
      Eigen::Matrix2d Rwb = rot.toRotationMatrix();

      lm_data = Rwb * meas.at(i)->GetMeaData().topLeftCorner(2, 2) +
                twb.replicate(1, 2);

      int id = SemanticMap::GetInstance().GetMapLandmarkNum(
          SEMANTIC_TYPE_PARKING_SLOT);
      SemanticLandmark::Ptr slot_lm =
          std::make_shared<ParkingSlotLandmark>(id, lm_data.data());
      slot_lm->AddSemanticMea(meas.at(i)->GetMeaTimestamp(), mea_pose,
                              meas.at(i));
      SemanticMap::GetInstance().AddLandmark(SEMANTIC_TYPE_PARKING_SLOT,
                                             slot_lm);
    }
  }
}
}  // namespace apa_slam
