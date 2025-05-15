/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/sim_data_loader.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Thursday, May 8th 2025, 1:10:57 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "sim_data_loader.h"

namespace apa_slam {
SimDataLoader::SimDataLoader() {}

std::vector<SemanticMea::Ptr> SimDataLoader::GetSemanticMeas(
    const double timestamp) {
  return _semantic_mea.at(timestamp);
}
std::vector<KinematicMea::Ptr> SimDataLoader::GetKinematicMeas(
    const double timestamp) {
  return _kinematic_mea.at(timestamp);
}

bool SimDataLoader::PopOutMea(ReplaySensorType& type, double& ts) {
  if (_mea_it == _mea_seq.end()) {
    return false;
  }

  type = _mea_it->second.at(_mea_id);
  ts = _mea_it->first;
  if (_mea_id == _mea_it->second.size() - 1) {
    _mea_id = 0;
    ++_mea_it;
  } else {
    ++_mea_id;
  }

  return true;
}

void SimDataLoader::LoadDataSet(const std::string& dataset_path) {
  const std::string pose_path = dataset_path + "/pose.txt";
  const std::string map_path = dataset_path + "/slot_map.txt";

  load_dataset_pose(pose_path);
  load_dataset_map(map_path);
  generate_semantic_meas();
  _mea_it = _mea_seq.begin();
}

void SimDataLoader::generate_semantic_meas() {
  const int interval = 10;
  auto it = _kinematic_mea.begin();
  auto& semantic_map = SemanticMap::GetInstance();
  auto slot_map = semantic_map.GetMap(SEMANTIC_TYPE_PARKING_SLOT);
  auto& params = ApaParameters::GetInstance().GetSimulationParameters();
  while (it != _kinematic_mea.end()) {
    double ts = it->first;
    Pose pose = _gt_pose.at(ts);
    Eigen::Vector2d twb(pose.x, pose.y);
    Eigen::Rotation2Dd rot(pose.yaw);
    Eigen::Matrix2d Rwb = rot.toRotationMatrix();

    Eigen::Matrix2d Rbw = Rwb.transpose();
    Eigen::Vector2d tbw = -Rwb.transpose() * twb;

    for (auto it_slot = slot_map.begin(); it_slot != slot_map.end();
         ++it_slot) {
      Eigen::MatrixXd lm_data = it_slot->second->GetLandmarkData();
      Eigen::Vector2d corner0 = lm_data.col(0).head(2);
      Eigen::Vector2d corner1 = lm_data.col(1).head(2);
      Eigen::Vector2d corner_c = .5f * (corner0 + corner1);

      if ((corner_c - twb).norm() > params.perception_sensing_range) {
        continue;
      }

      Eigen::MatrixXd mea =
          Eigen::MatrixXd::Zero(DATA_ROWS_PARKING_SLOT, DATA_COLS_PARKING_SLOT);
      for (int i = 0; i < DATA_COLS_PARKING_SLOT; ++i) {
        Eigen::Vector2d point_w = lm_data.col(i).head(2);
        Eigen::Vector2d point_b = Rbw * point_w + tbw;
        mea.col(i).head(2) = point_b;
      }

      SemanticMea::Ptr slot_mea =
          std::make_shared<ParkingSlotMea>(ts, mea.data());

      slot_mea->AddNoise();
      _semantic_mea[ts].push_back(slot_mea);
    }

    if (_semantic_mea.count(ts)) {
      _mea_seq[ts].push_back(ReplaySensorType::REPLAY_TYPE_SEMANTIC);
    }

    bool come_end = false;
    for (int i = 0; i < interval; ++i) {
      ++it;
      if (it == _kinematic_mea.end()) {
        come_end = true;
        break;
      }
    }

    if (come_end) {
      break;
    }
  }
}

void SimDataLoader::load_dataset_pose(const std::string& pose_file) {
  std::ifstream fpose;
  fpose.open(pose_file, std::ios::in);
  std::string line;
  while (getline(fpose, line)) {
    std::stringstream ss(line);
    Pose pose;
    double timestamp;
    double velocity_x;
    double velocity_y;
    double angular_velocity;
    ss >> timestamp >> pose.x >> pose.y >> pose.yaw >> velocity_x >>
        velocity_y >> angular_velocity;
    _gt_pose[timestamp] = pose;
    Eigen::VectorXd odo_mea_data = Eigen::VectorXd::Zero(DATA_ROWS_ODO);
    odo_mea_data[0] = std::hypotf(velocity_x, velocity_y);
    odo_mea_data[1] = angular_velocity;
    KinematicMea::Ptr odo_mea =
        std::make_shared<OdoMea>(timestamp, odo_mea_data.data());

    odo_mea->AddNoise();

    _mea_seq[timestamp].push_back(ReplaySensorType::REPLAY_TYPE_KINEMATIC);
    _kinematic_mea[timestamp].push_back(odo_mea);
    _gt_pose[timestamp] = pose;
  }
  fpose.close();
}

void SimDataLoader::load_dataset_map(const std::string& map_file) {
  auto& semantic_map = SemanticMap::GetInstance();
  std::ifstream fmap;
  fmap.open(map_file, std::ios::in);
  std::string line;
  while (getline(fmap, line)) {
    std::stringstream ss(line);
    int id;
    Eigen::MatrixXd slot_data =
        Eigen::MatrixXd::Zero(DATA_ROWS_PARKING_SLOT, DATA_COLS_PARKING_SLOT);
    ss >> id;
    for (int i = 0; i < DATA_COLS_PARKING_SLOT; ++i) {
      for (int j = 0; j < DATA_ROWS_PARKING_SLOT; ++j) {
        ss >> slot_data(j, i);
      }
    }

    SemanticLandmark::Ptr slot_landmark =
        std::make_shared<ParkingSlotLandmark>(id, slot_data.data());
    semantic_map.AddLandmark(slot_landmark->GetSemanticType(), slot_landmark);
  }
  fmap.close();
}
}  // namespace apa_slam
