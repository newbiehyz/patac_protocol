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
  if (SemanticMap::GetInstance().HasMap(
          SensorType::SEMANTIC_TYPE_PARKING_SLOT)) {
    auto matching_local_map = matching_with_local_map(meas, pose);
    auto matching = fill_matching(matching_local_map);

    _last_meas = meas;
    _last_pose = pose;
    _last_map_matching = matching;
    return matching;
  } else {
    auto matching_last_mea = matching_with_last_mea(meas, pose);
    auto matching = fill_matching(matching_last_mea);
    _last_meas = meas;
    _last_pose = pose;

    _last_map_matching = matching;
    return matching;
  }
}

std::vector<int> ParkingSlotTracker::matching_with_last_mea(
    const std::vector<SemanticMea::Ptr>& meas, const Pose& pose) {
  std::vector<int> matching;
  matching.resize(meas.size());
  std::fill(matching.begin(), matching.end(), -1);
  const double max_cost = 10000.0;
  const double dist_thresh = ApaParameters::GetInstance()
                                 .GetEstimatorParamters()
                                 .slot_matching_dist_thresh;
  const double angle_thresh = ApaParameters::GetInstance()
                                  .GetEstimatorParamters()
                                  .slot_matching_angle_thresh;
  if (_last_meas.empty()) {
    for (size_t i = 0; i < meas.size(); ++i) {
      matching.at(i) = i;
    }

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
        cost = matching_cost[0] > dist_thresh ? max_cost : cost;
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
        matching.at(cur_mea_id) = _last_map_matching.at(last_mea_id);
      }
    }
  }

  return matching;
}

std::vector<int> ParkingSlotTracker::fill_matching(
    const std::vector<int>& matching) {
  int num_landmarks =
      SemanticMap::GetInstance().GetMapLandmarkNum(SEMANTIC_TYPE_PARKING_SLOT);
  std::vector<int> matching_filled;
  for (size_t i = 0; i < matching.size(); ++i) {
    if (matching.at(i) == -1) {
      matching_filled.push_back(num_landmarks);
      ++num_landmarks;
    } else {
      matching_filled.push_back(matching.at(i));
    }
  }

  return matching_filled;
}

std::vector<int> ParkingSlotTracker::get_consist_matching(
    const std::vector<int>& matching_mea,
    const std::vector<int>& matching_map) {
  std::cout << "matching_last_mea******:\n";
  for (size_t i = 0; i < matching_mea.size(); ++i) {
    std::cout << matching_mea.at(i) << " ";
  }
  std::cout << std::endl;
  if (!matching_map.empty()) {
    std::cout << "matching_local_map******:\n";
    for (size_t i = 0; i < matching_map.size(); ++i) {
      std::cout << matching_map.at(i) << " ";
    }
    std::cout << std::endl;
  }

  std::vector<int> matching;
  matching.resize(matching_mea.size());
  int num_landmarks =
      SemanticMap::GetInstance().GetMapLandmarkNum(SEMANTIC_TYPE_PARKING_SLOT);
  for (size_t i = 0; i < matching_mea.size(); ++i) {
    if (matching_map.empty()) {
      if (matching_mea.at(i) == -1) {
        matching.at(i) = num_landmarks;
        ++num_landmarks;
      } else {
        matching.at(i) = matching_mea.at(i);
      }
    } else {
      if (matching_mea.at(i) == matching_map.at(i)) {
        if (matching_mea.at(i) == -1) {
          matching.at(i) = num_landmarks;
          ++num_landmarks;
        } else {
          matching.at(i) = matching_mea.at(i);
        }
      } else {
        if (matching_map.at(i) != -1) {
          matching.at(i) = matching_map.at(i);
        } else {
          matching.at(i) = num_landmarks;
          ++num_landmarks;
        }
      }
    }
  }

  std::cout << "matching#######:\n";
  for (size_t i = 0; i < matching.size(); ++i) {
    std::cout << matching.at(i) << " ";
  }
  std::cout << std::endl;

  return matching;
}

std::vector<int> ParkingSlotTracker::matching_with_local_map(
    const std::vector<SemanticMea::Ptr>& meas, const Pose& pose) {
  std::vector<int> matching;
  matching.resize(meas.size());
  std::fill(matching.begin(), matching.end(), -1);
  const double max_cost = 10000.0;
  const double dist_thresh = ApaParameters::GetInstance()
                                 .GetEstimatorParamters()
                                 .slot_matching_dist_thresh;
  const double angle_thresh = ApaParameters::GetInstance()
                                  .GetEstimatorParamters()
                                  .slot_matching_angle_thresh;

  std::vector<SemanticLandmark::Ptr> local_map;
  Eigen::VectorXd pose_eigen = Eigen::VectorXd::Zero(3);
  pose_eigen.x() = pose.x;
  pose_eigen.y() = pose.y;
  pose_eigen.z() = pose.yaw;

  if (SemanticMap::GetInstance().GetFullLocalMap(
          SensorType::SEMANTIC_TYPE_PARKING_SLOT, pose, local_map)) {
    if (local_map.empty()) {
      for (size_t i = 0; i < meas.size(); ++i) {
        matching.at(i) = i;
      }
    } else {
      std::vector<int> unassigned_ref_idx, unassigned_new_idx;
      std::vector<std::vector<double>> cost_vector(meas.size());

      for (int i = 0; i < meas.size(); ++i) {
        cost_vector.at(i).resize(local_map.size());
        for (int j = 0; j < local_map.size(); ++j) {
          Eigen::VectorXd matching_cost =
              local_map.at(j)->ComputeMatchingResidual(pose_eigen, meas.at(i));
          double cost =
              matching_cost[1] > angle_thresh ? max_cost : matching_cost[0];
          cost = matching_cost[0] > dist_thresh ? max_cost : cost;
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
          matching.at(cur_mea_id) = local_map.at(last_mea_id)->GetId();
        }

        const double second_small_dist = 0.3;

        if (cost_vector.at(cur_mea_id).size() >= 2) {
          std::pair<double, double> smallest =
              get_two_smallest(cost_vector.at(cur_mea_id));
          if (smallest.first < max_cost && smallest.second < max_cost) {
            if (fabs(smallest.first - smallest.second) < second_small_dist) {
              matching.at(cur_mea_id) = -2;
            }
          }
        }
      }
    }
  }

  return matching;
}

std::pair<double, double> ParkingSlotTracker::get_two_smallest(
    const std::vector<double>& vec) {
  double smallest = std::numeric_limits<double>::max();
  double second_smallest = std::numeric_limits<double>::max();

  for (double val : vec) {
    if (val < smallest) {
      second_smallest = smallest;
      smallest = val;
    } else if (val < second_smallest) {
      second_smallest = val;
    }
  }

  return {smallest, second_smallest};
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
      Rwb1 * mea1->GetMeaData().topLeftCorner(2, 2) + twb1.replicate(1, 2);

  distance[0] =
      ((pt0_w.col(0) + pt0_w.col(1) - pt1_w.col(0) - pt1_w.col(1)) * .5).norm();

  Eigen::Vector2d dir0 = (pt0_w.col(0) - pt0_w.col(1)).normalized();
  Eigen::Vector2d dir1 = (pt1_w.col(0) - pt1_w.col(1)).normalized();

  double angular_distance = std::acos(fabs(dir0.dot(dir1)));

  distance[1] = angular_distance;  // radiant

  return distance;
}

}  // namespace apa_slam
