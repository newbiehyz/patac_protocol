/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/src/parking_slot_tracker.cc Path:
 * /home/yukan/Documents/work/local_mapping/src Created Date: Friday, May 9th
 * 2025, 4:04:47 pm Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include "parking_slot_tracker.h"

namespace apa_slam {
ParkingSlotTracker::ParkingSlotTracker()
    : TrackerBase(SEMANTIC_TYPE_PARKING_COLUMN) {}

std::vector<int> ParkingSlotTracker::HungarianMatching(
    const std::vector<SemanticMea::Ptr>& meas, const Pose& pose) {
  std::vector<int> map_matching;
  map_matching.resize(meas.size());
  std::fill(map_matching.begin(), map_matching.end(), -1);
  const double max_cost = 10000.0;
  const double dist_thresh = ApaParameters::GetInstance()
                                 .GetEstimatorParamters()
                                 .slot_matching_dist_thresh;
  const double angle_thresh = ApaParameters::GetInstance()
                                  .GetEstimatorParamters()
                                  .slot_matching_angle_thresh;
  if (_last_meas.empty()) {
    _last_meas = meas;
    _last_pose = pose;

    for (size_t i = 0; i < meas.size(); ++i) {
      map_matching.at(i) = i;
    }

    _last_map_matching = map_matching;

  } else {
    std::vector<int> unassigned_ref_idx, unassigned_new_idx;
    std::vector<std::vector<double>> cost_vector(meas.size());
    for (int i = 0; i < meas.size(); ++i) {
      cost_vector.at(i).resize(_last_meas.size());
      for (int j = 0; j < _last_meas.size(); ++j) {
        Eigen::VectorXd matching_cost = compute_matching_distance(
            meas.at(i), pose, _last_meas.at(j), _last_pose);

        double cost =
            matching_cost[1] > angle_thresh ? max_cost : matching_cost[0];
        cost_vector.at(i).at(j) = cost;
      }
    }
    std::vector<int> correspondences;
    std::fill(correspondences.begin(), correspondences.end(), -1);
    HungarianOptimizer hungarian_optimizer(cost_vector);
    hungarian_optimizer.Minimize(&unassigned_ref_idx, &unassigned_new_idx);

    for (int i = 0; i < unassigned_ref_idx.size(); ++i) {
      int cur_mea_id = unassigned_ref_idx.at(i);
      int last_mea_id = unassigned_new_idx.at(i);
      if (cost_vector[cur_mea_id][last_mea_id] < dist_thresh) {
        map_matching.at(cur_mea_id) = _last_map_matching.at(last_mea_id);
      }
    }

    int num_landmarks = SemanticMap::GetInstance().GetMapLandmarkNum(
        SEMANTIC_TYPE_PARKING_SLOT);
    for (size_t i = 0; i < map_matching.size(); ++i) {
      if (map_matching.at(i) == -1) {
        map_matching.at(i) = num_landmarks;
        ++num_landmarks;
      }
    }
  }

  return map_matching;
}

Eigen::VectorXd ParkingSlotTracker::compute_matching_distance(
    const SemanticMea::Ptr mea0, const Pose& pose0, const SemanticMea::Ptr mea1,
    const Pose& pose1) {
  Eigen::VectorXd distance = Eigen::VectorXd::Zero(2);

  Eigen::Vector2d twb0(pose0.x, pose0.y);
  Eigen::Rotation2Dd rot0(pose0.yaw);
  Eigen::Matrix2d Rwb0 = rot0.toRotationMatrix();

  Eigen::Vector2d twb1(pose1.x, pose1.y);
  Eigen::Rotation2Dd rot1(pose1.yaw);
  Eigen::Matrix2d Rwb1 = rot1.toRotationMatrix();

  Eigen::Matrix2d pt0_w =
      Rwb0 * mea0->GetMeaData().topLeftCorner(2, 2) + twb0.replicate(1, 2);
  Eigen::Matrix2d pt1_w =
      Rwb0 * mea1->GetMeaData().topLeftCorner(2, 2) + twb1.replicate(1, 2);

  distance[0] =
      ((pt0_w.col(0) + pt0_w.col(1) - pt1_w.col(0) - pt1_w.col(1)) * .5).norm();

  Eigen::Vector2d dir0 = (pt0_w.col(0) - pt0_w.col(1)).normalized();
  Eigen::Vector2d dir1 = (pt1_w.col(0) - pt1_w.col(1)).normalized();

  double angular_distance = std::acos(fabs(dir0.dot(dir1)));

  distance[1] = angular_distance;  // radiant

  return distance;
}

}  // namespace apa_slam
