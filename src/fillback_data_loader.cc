/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/src/fillback_data_loader.cc Path:
 * /home/yukan/Documents/work/local_mapping/src Created Date: Monday, May 19th
 * 2025, 9:27:12 am Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */


#include "fillback_data_loader.h"

namespace apa_slam {
FillbackDataLoader::FillbackDataLoader() {}

void FillbackDataLoader::LoadDataSet(const std::string& dataset_path) {
  const std::string odo_file = dataset_path + "/CANData.json";
  // load_odo_meas(odo_file);

  const std::string pose_file = dataset_path + "/DR_POSE.json";
  // load_pose_odo_meas(pose_file, odo_file);
  load_pose_odo_meas2(pose_file);
  std::cout << "load odo done\n";
  const std::string semantci_mea_file = dataset_path + "/RDinfo.json";
  load_semantic_meas(semantci_mea_file);
  // double start_ts = _semantic_mea.begin()->first - 5.0;

  _mea_it = _mea_seq.begin();
}
bool FillbackDataLoader::PopOutMea(ReplaySensorType& type,
                                   long long& arriving_ts,
                                   long long& sensor_ts) {
  if (_mea_it == _mea_seq.end()) {
    return false;
  }
  arriving_ts = _mea_it->first;
  type = _mea_it->second.at(_mea_id).first;
  sensor_ts = _mea_it->second.at(_mea_id).second;
  if (_mea_id == _mea_it->second.size() - 1) {
    _mea_id = 0;
    ++_mea_it;
  } else {
    ++_mea_id;
  }

  return true;
}

std::vector<SemanticMea::Ptr> FillbackDataLoader::GetSemanticMeas(
    const long long timestamp) {
  return _semantic_mea.at(timestamp);
}
std::vector<KinematicMea::Ptr> FillbackDataLoader::GetKinematicMeas(
    const long long timestamp) {
  return _kinematic_mea.at(timestamp);
}

Eigen::Vector2d FillbackDataLoader::ConvertUvToVehicle(
    const Eigen::Vector2d& uv) {
  float REAR_AXEL_TO_CENTER =
      (APA_VEHICLE_LENGTH / 2) - APA_REAR_AXLE_CENTER_VEHICLE_REAR;

  float x = uv.x() - APA_BIRD_VIEW_HEIGHT / 2;
  float y = APA_BIRD_VIEW_HEIGHT / 2 - uv.y() +
            (REAR_AXEL_TO_CENTER / APA_LR_BIRD_PIXECL_2_WORLD);

  Eigen::Vector2d corner_v;

  corner_v.x() = y * APA_LR_BIRD_PIXECL_2_WORLD;
  corner_v.y() = -x * APA_LR_BIRD_PIXECL_2_WORLD;

  // corner_v.x() = x * LR_BIRD_PIXECL_2_WORLD;
  // corner_v.y() = y * LR_BIRD_PIXECL_2_WORLD;

  corner_v /= 1000.0f;

  return corner_v;
}

double FillbackDataLoader::angle_diff(double angle1, double angle2) {
  double diff = angle1 - angle2;
  while (diff > M_PI) diff -= 2 * M_PI;
  while (diff < -M_PI) diff += 2 * M_PI;
  return diff;
}


void FillbackDataLoader::load_pose_odo_meas2(const std::string& pose_file) {
  std::ifstream file(pose_file);
  json j;
  file >> j;  // Parse JSON
  std::map<long long, Eigen::Vector3d> pose_data;
  long long last_ts = -1;
  for (const auto& item : j) {
    double x = item["x"];
    double y = item["y"];
    long long timestamp = item["timeStamp"];

    double xx = y;
    double yy = -x;
    xx /= 1000.0f;
    yy /= 1000.0f;
    double yaw = item["canAng"];
    yaw = -yaw;
    yaw = yaw / 180.0 * M_PI;

    if (timestamp <= 1e-4) {
      continue;
    }
    if (timestamp <= last_ts) {
      continue;
    }

    if (timestamp < ApaParameters::GetInstance()
                        .GetDatasetParameters()
                        .min_dataset_timestamp ||
        timestamp > ApaParameters::GetInstance()
                        .GetDatasetParameters()
                        .max_dataset_timestamp) {
      continue;
    }
    last_ts = timestamp;
    pose_data[timestamp] = Eigen::Vector3d(xx, yy, yaw);
  }
  file.close();
  _pose_data = pose_data;
  std::set<long long> can_timestamp;

  auto it0 = pose_data.begin();
  auto it1 = it0;
  ++it1;
  while (it1 != pose_data.end()) {
    long long t0 = it0->first;
    long long t1 = it1->first;
    double dt = 0.01;
    while ((t0 + dt) < t1) {
      can_timestamp.insert(
          t0 +
          dt / ApaParameters::GetInstance().GetEstimatorParamters().time_scale);
      dt += 0.01;
    }

    ++it0;
    ++it1;
  }
  std::cout << "generating v and w\n";
  std::vector<std::pair<long long, Eigen::Vector3d>> pose_can;
  for (auto it_ts = can_timestamp.begin(); it_ts != can_timestamp.end();
       ++it_ts) {
    long long timestamp = *it_ts;
    auto it_upper = pose_data.lower_bound(timestamp);
    auto it_lower = it_upper;
    --it_lower;

    Eigen::Vector2d twb0 = it_lower->second.head(2);
    Eigen::Vector2d twb1 = it_upper->second.head(2);
    Eigen::Matrix3d Rwb0 =
        Eigen::AngleAxisd(it_lower->second.z(), Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    Eigen::Matrix3d Rwb1 =
        Eigen::AngleAxisd(it_upper->second.z(), Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    Eigen::Quaterniond qwb0(Rwb0);
    Eigen::Quaterniond qwb1(Rwb1);

    double ratio = static_cast<double>(timestamp - it_lower->first) /
                   static_cast<double>(it_upper->first - it_lower->first);
    Eigen::Vector2d twb = twb0 + ratio * (twb1 - twb0);
    Eigen::Quaterniond qwb = qwb0.slerp(ratio, qwb1);

    Eigen::AngleAxisd axang;
    axang.fromRotationMatrix(qwb.toRotationMatrix());
    double yaw = axang.axis().z() > 0 ? axang.angle() : -axang.angle();
    pose_can.push_back({timestamp, Eigen::Vector3d(twb.x(), twb.y(), yaw)});
  }

  for (size_t i = 1; i < pose_can.size() - 2; ++i) {
    int j0 = i - 1;
    int j1 = i + 1;
    double ts0 = pose_can.at(j0).first;
    double ts1 = pose_can.at(j1).first;

    Eigen::Vector2d twb0 = pose_can.at(j0).second.head(2);
    Eigen::Vector2d twb1 = pose_can.at(j1).second.head(2);

    double yaw0 = pose_can.at(j0).second.z();
    double yaw1 = pose_can.at(j1).second.z();

    Eigen::Matrix3d Rwb0 =
        Eigen::AngleAxisd(yaw0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d Rwb1 =
        Eigen::AngleAxisd(yaw1, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    double diff = angle_diff(yaw1, yaw0);
    double dt = static_cast<double>(ts1 - ts0) * 0.001;
    double w = diff / dt;
    double v = (twb0 - twb1).norm() / dt;
    // v *= 0.95;

    Eigen::Vector2d dir0 = Rwb0.col(0).head(2);
    Eigen::Vector2d dir = (twb1 - twb0).normalized();
    if (dir.dot(dir0) < 0) {
      v *= -1;
    }

    Eigen::VectorXd odo_mea_data = Eigen::VectorXd::Zero(DATA_ROWS_ODO);
    odo_mea_data[0] = v;
    odo_mea_data[1] = w;
    KinematicMea::Ptr odo_mea =
        std::make_shared<OdoMea>(pose_can.at(i).first, odo_mea_data.data());
    _mea_seq[pose_can.at(i).first].push_back(
        {ReplaySensorType::REPLAY_TYPE_KINEMATIC, pose_can.at(i).first});
    _kinematic_mea[pose_can.at(i).first].push_back(odo_mea);
  }

  std::cout << "========\n";
}

void FillbackDataLoader::load_pose_odo_meas(const std::string& pose_file,
                                            const std::string& can_file) {
  std::ifstream file(pose_file);
  json j;
  file >> j;  // Parse JSON
  std::map<long long, Eigen::Vector3d> pose_data;
  for (const auto& item : j) {
    double x = item["x"];
    double y = item["y"];
    double xx = y;
    double yy = -x;
    xx /= 1000.0f;
    yy /= 1000.0f;
    double yaw = item["canAng"];
    // yaw = (90.0 - yaw) / 180.0 * M_PI;
    yaw = -yaw;
    yaw = yaw / 180.0 * M_PI;

    long long timestamp = item["timeStamp"];
    // double timestamp_d =
    //     static_cast<double>(timestamp) * static_cast<double>(1e-3);
    pose_data[timestamp] = Eigen::Vector3d(xx, yy, yaw);
    // std::cout <<  "[" << xx << " , " << yy << " , " << yaw << "],"<<
    // std::endl;
  }
  file.close();
  _pose_data = pose_data;
  std::set<long long> can_timestamp;

  std::ifstream file_can(can_file);
  json j_can;
  file_can >> j_can;  // Parse JSON

  for (const auto& item : j_can) {
    if (item.contains("timestamp") && item.contains("VehSpdAvgNDrvn") &&
        item.contains("TARS_TransActRng") && item.contains("StrWhAng")) {
      long long timestamp = item["timestamp"];

      if (timestamp > pose_data.begin()->first &&
          timestamp < pose_data.rbegin()->first) {
        can_timestamp.insert(timestamp);
      }
    }
  }
  file_can.close();

  std::vector<std::pair<long long, Eigen::Vector3d>> pose_can;

  for (auto it_ts = can_timestamp.begin(); it_ts != can_timestamp.end();
       ++it_ts) {
    double timestamp = *it_ts;
    auto it_upper = pose_data.lower_bound(timestamp);
    auto it_lower = it_upper;
    --it_lower;

    Eigen::Vector2d twb0 = it_lower->second.head(2);
    Eigen::Vector2d twb1 = it_upper->second.head(2);
    Eigen::Matrix3d Rwb0 =
        Eigen::AngleAxisd(it_lower->second.z(), Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    Eigen::Matrix3d Rwb1 =
        Eigen::AngleAxisd(it_upper->second.z(), Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    Eigen::Quaterniond qwb0(Rwb0);
    Eigen::Quaterniond qwb1(Rwb1);

    double ratio = static_cast<double>(timestamp - it_lower->first) /
                   static_cast<double>(it_upper->first - it_lower->first);
    Eigen::Vector2d twb = twb0 + ratio * (twb1 - twb0);
    Eigen::Quaterniond qwb = qwb0.slerp(ratio, qwb1);

    Eigen::AngleAxisd axang;
    axang.fromRotationMatrix(qwb.toRotationMatrix());
    double yaw = axang.axis().z() > 0 ? axang.angle() : -axang.angle();
    pose_can.push_back({timestamp, Eigen::Vector3d(twb.x(), twb.y(), yaw)});
  }

  for (size_t i = 1; i < pose_can.size() - 2; ++i) {
    int j0 = i - 1;
    int j1 = i + 1;
    double ts0 = pose_can.at(j0).first;
    double ts1 = pose_can.at(j1).first;

    Eigen::Vector2d twb0 = pose_can.at(j0).second.head(2);
    Eigen::Vector2d twb1 = pose_can.at(j1).second.head(2);

    double yaw0 = pose_can.at(j0).second.z();
    double yaw1 = pose_can.at(j1).second.z();

    Eigen::Matrix3d Rwb0 =
        Eigen::AngleAxisd(yaw0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d Rwb1 =
        Eigen::AngleAxisd(yaw1, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    double diff = angle_diff(yaw1, yaw0);
    double dt = static_cast<double>(ts1 - ts0) * 0.001;
    double w = diff / dt;
    double v = (twb0 - twb1).norm() / dt;
    // v *= 0.95;

    Eigen::Vector2d dir0 = Rwb0.col(0).head(2);
    Eigen::Vector2d dir = (twb1 - twb0).normalized();
    if (dir.dot(dir0) < 0) {
      v *= -1;
    }

    Eigen::VectorXd odo_mea_data = Eigen::VectorXd::Zero(DATA_ROWS_ODO);
    odo_mea_data[0] = v;
    odo_mea_data[1] = w;
    KinematicMea::Ptr odo_mea =
        std::make_shared<OdoMea>(pose_can.at(i).first, odo_mea_data.data());
    _mea_seq[pose_can.at(i).first].push_back(
        {ReplaySensorType::REPLAY_TYPE_KINEMATIC, pose_can.at(i).first});
    _kinematic_mea[pose_can.at(i).first].push_back(odo_mea);
  }

  std::cout << "========\n";
}



Eigen::VectorXd FillbackDataLoader::interpolate_pose(
    const long long timestamp,
    const std::map<long long, Eigen::Vector3d>& pose_data) {
  auto it_lower = pose_data.lower_bound(timestamp);
  auto it_upper = it_lower;
  --it_lower;

  double ratio = static_cast<double>(timestamp - it_lower->first) /
                 static_cast<double>(it_upper->first - it_lower->first);
  Eigen::Vector2d twb_lower = it_lower->second.head(2);
  Eigen::Vector2d twb_upper = it_upper->second.head(2);

  Eigen::Matrix3d Rwb_lower =
      Eigen::AngleAxisd(it_lower->second.z(), Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  Eigen::Matrix3d Rwb_upper =
      Eigen::AngleAxisd(it_upper->second.z(), Eigen::Vector3d::UnitZ())
          .toRotationMatrix();

  Eigen::Quaterniond qwb_lower(Rwb_lower);
  Eigen::Quaterniond qwb_upper(Rwb_upper);

  Eigen::Vector2d twb = twb_lower + ratio * (twb_upper - twb_lower);
  Eigen::Quaterniond qwb = qwb_lower.slerp(ratio, qwb_upper);
  Eigen::AngleAxisd axang_wb;
  axang_wb.fromRotationMatrix(qwb.toRotationMatrix());
  double theta = axang_wb.axis().z() > 0 ? axang_wb.angle() : -axang_wb.angle();

  Eigen::VectorXd pose = Eigen::VectorXd::Zero(3);
  pose.head(2) = twb;
  pose.z() = theta;

  return pose;
}
FillbackDataLoader& FillbackDataLoader::GetInstance() {
  static FillbackDataLoader instance;
  return instance;
}
void FillbackDataLoader::load_semantic_meas(
    const std::string& semantic_mea_file) {
  std::ifstream file(semantic_mea_file);
  json j;
  file >> j;  // Parse JSON
  long long last_timestamp = -1;
  for (const auto& item : j) {
    long long timestamp = item["frameTimeStampNs"];

    if (timestamp < _kinematic_mea.begin()->first ||
        timestamp > _kinematic_mea.rbegin()->first) {
      continue;
    }

    if (timestamp == last_timestamp) {
      last_timestamp = timestamp;
      continue;
    }

    if (timestamp > 0) {
      auto quadParkingSlotList = item["quadParkingSlotList"];
      Eigen::VectorXd pose = interpolate_pose(timestamp, _pose_data);
      Eigen::Vector2d twb = pose.head(2);
      Eigen::Rotation2Dd rot_wb(pose.z());
      Eigen::Matrix2d Rwb = rot_wb.toRotationMatrix();

      if (quadParkingSlotList.size() > 0) {
        if (ApaParameters::GetInstance()
                .GetEstimatorParamters()
                .export_debug_file) {
          std::ofstream fout_pose("/home/yukan/Documents/pose.txt",
                                  std::ios::out | std::ios::app);
          fout_pose << pose.x() << " " << pose.y() << " " << pose.z()
                    << std::endl;
          fout_pose.close();
        }
        int id = 0;
        for (const auto& slot : quadParkingSlotList) {
          Eigen::Vector2d corner_l_uv(slot["tl"]["x"], slot["tl"]["y"]);
          Eigen::Vector2d corner_r_uv(slot["tr"]["x"], slot["tr"]["y"]);
          Eigen::Vector2d corner_l = ConvertUvToVehicle(corner_l_uv);
          Eigen::Vector2d corner_r = ConvertUvToVehicle(corner_r_uv);
          Eigen::Vector2d dir = corner_r - corner_l;
          dir.normalize();
          corner_l += dir * 0.12;
          corner_r -= dir * 0.12;

          // std::cout << "slot width: " << (corner_l - corner_r).norm() <<
          // std::endl;

          Eigen::MatrixXd mea = Eigen::MatrixXd::Zero(DATA_ROWS_PARKING_SLOT,
                                                      DATA_COLS_PARKING_SLOT);
          mea.col(0).head(2) = corner_l;
          mea.col(1).head(2) = corner_r;

          if (timestamp > _pose_data.begin()->first &&
              timestamp < _pose_data.rbegin()->first) {
            Eigen::Vector2d corner_l_w = Rwb * corner_l + twb;
            Eigen::Vector2d corner_r_w = Rwb * corner_r + twb;
            if (ApaParameters::GetInstance()
                    .GetEstimatorParamters()
                    .export_debug_file) {
              std::ofstream fout_lm("/home/yukan/Documents/mapping.txt",
                                    std::ios::out | std::ios::app);
              fout_lm << corner_l_w.x() << " " << corner_l_w.y() << " "
                      << corner_r_w.x() << " " << corner_r_w.y();
              if (id = quadParkingSlotList.size() - 1) {
                fout_lm << " ";
              } else {
                fout_lm << std::endl;
              }

              fout_lm.close();
            }
          }

          SemanticMea::Ptr slot_mea =
              std::make_shared<ParkingSlotMea>(timestamp, mea.data());
          _semantic_mea[timestamp].push_back(slot_mea);
          ++id;
        }

        if (_semantic_mea.count(timestamp)) {
          _mea_seq[timestamp + ApaParameters::GetInstance()
                                       .GetDatasetParameters()
                                       .timedelay /
                                   ApaParameters::GetInstance()
                                       .GetEstimatorParamters()
                                       .time_scale]
              .push_back({ReplaySensorType::REPLAY_TYPE_SEMANTIC, timestamp});
        }
      }
    }
    last_timestamp = timestamp;
  }
}
}  // namespace apa_slam
