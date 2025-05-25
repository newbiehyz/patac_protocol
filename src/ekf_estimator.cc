/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/ekf_estimator.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Friday, May 9th 2025, 9:28:19 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "ekf_estimator.h"

namespace apa_slam {
EkfEstimator::EkfEstimator() {}

void EkfEstimator::Init() {
  TrackerBase::Ptr parking_slot_tracker =
      std::make_shared<ParkingSlotTracker>();
  _tracker_pools.insert({SEMANTIC_TYPE_PARKING_SLOT, parking_slot_tracker});

  SemanticMap::GetInstance().ClearMap();

  EKFManagement::GetInstance().Init();
}

bool EkfEstimator::GetLatestVechileState(double &timestamp,
                                         Eigen::VectorXd &mean,
                                         Eigen::MatrixXd &cov) {
  return EKFManagement::GetInstance().GetLatestVechileState(timestamp, mean,
                                                            cov);
}

bool EkfEstimator::ProcDrPose(double ts, const Pose &pose, double &ts_out,
                              double &v_out, double &w_out) {
  if (_dr_pose.size() < 3) {
    _dr_pose.push_back(pose);
    _dr_timestamp.push_back(ts);
    return false;
  } else {
    double ts0 = _dr_timestamp[0];
    double ts1 = _dr_timestamp[2];

    Eigen::Vector2d twb0(_dr_pose[0].x, _dr_pose[0].y);
    Eigen::Vector2d twb1(_dr_pose[2].x, _dr_pose[2].y);
    Eigen::Matrix3d Rwb0 =
        Eigen::AngleAxisd(_dr_pose[0].yaw, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    Eigen::Matrix3d Rwb1 =
        Eigen::AngleAxisd(_dr_pose[2].yaw, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();

    double diff = angle_diff(_dr_pose[0].yaw, _dr_pose[2].yaw);

    w_out = diff / (ts1 - ts0);
    v_out = (twb0 - twb1).norm() / (ts1 - ts0);
    ts_out = _dr_timestamp[1];

    Eigen::Vector2d dir0 = Rwb0.col(0).head(2);
    Eigen::Vector2d dir = (twb1 - twb0).normalized();
    if (dir.dot(dir0) < 0) {
      v_out *= -1;
    }

    std::swap(_dr_timestamp[0], _dr_timestamp[1]);
    std::swap(_dr_timestamp[1], _dr_timestamp[2]);

    std::swap(_dr_pose[0], _dr_pose[1]);
    std::swap(_dr_pose[1], _dr_pose[2]);

    _dr_pose[2] = pose;
    _dr_timestamp[2] = ts;

    return true;
  }
}

void EkfEstimator::Reset() {
  // clear map
  SemanticMap::GetInstance().ClearMap();
}

double EkfEstimator::angle_diff(double angle1, double angle2) {
  double diff = angle1 - angle2;
  while (diff > M_PI) diff -= 2 * M_PI;
  while (diff < -M_PI) diff += 2 * M_PI;
  return diff;
}

EkfEstimator &EkfEstimator::GetInstance() {
  static EkfEstimator instance;
  return instance;
}

void EkfEstimator::InputSemanticMea(
    const double ts, const std::vector<SemanticMea::Ptr> &semantic_meas) {
  auto start = std::chrono::steady_clock::now();

  if (!this->Initialized()) {
    return;
  }
  std::unordered_map<SensorType, std::vector<SemanticMea::Ptr>> meas_sorted;
  sort_semantic_meas(semantic_meas, meas_sorted);

  for (auto it = meas_sorted.begin(); it != meas_sorted.end(); ++it) {
    const SensorType mea_type = it->first;
    process_semantic_meas(mea_type, ts, it->second);
  }

  EKFManagement::GetInstance().Update(ts);

  auto end = std::chrono::steady_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
          .count();

  std::cout << "Update Duration: " << duration << " MicroSeconds" << std::endl;
}

void EkfEstimator::process_semantic_meas(
    const SensorType &type, const double ts,
    const std::vector<SemanticMea::Ptr> &parking_slot_meas) {
  Pose mea_pose;
  if (!get_pose(ts, mea_pose)) {
    return;
  }

  std::vector<int> matching_rs =
      _tracker_pools.at(type)->HungarianMatching(parking_slot_meas, mea_pose);
  if (ApaParameters::GetInstance().GetEstimatorParamters().export_debug_file) {
    std::ofstream fout_pose("/home/yukan/Documents/dr_pose.txt",
                            std::ios::out | std::ios::app);
    fout_pose << mea_pose.x << " " << mea_pose.y << " " << mea_pose.yaw
              << std::endl;
    fout_pose.close();
  }

  for (size_t i = 0; i < parking_slot_meas.size(); ++i) {
    Eigen::Vector2d twb(mea_pose.x, mea_pose.y);
    Eigen::Rotation2Dd rot(mea_pose.yaw);
    Eigen::Matrix2d Rwb = rot.toRotationMatrix();
    Eigen::Vector2d mea0 = parking_slot_meas.at(i)->GetMeaData().col(0).head(2);
    Eigen::Vector2d mea1 = parking_slot_meas.at(i)->GetMeaData().col(1).head(2);
    Eigen::Vector2d lm0 = Rwb * mea0 + twb;
    Eigen::Vector2d lm1 = Rwb * mea1 + twb;
    if (ApaParameters::GetInstance()
            .GetEstimatorParamters()
            .export_debug_file) {
      std::ofstream fout_projection("/home/yukan/Documents/dr_projection.txt",
                                    std::ios::out | std::ios::app);
      fout_projection << lm0.x() << " " << lm0.y() << " " << lm1.x() << " "
                      << lm1.y();
      if (i != parking_slot_meas.size() - 1) {
        fout_projection << " ";
      } else {
        fout_projection << std::endl;
      }
      fout_projection.close();
    }
  }

  std::cout << "Matching Log:\n";
  for (size_t i = 0; i < matching_rs.size(); ++i) {
    std::cout << matching_rs.at(i) << " ";
  }

  std::cout << std::endl;

  MapManagement::GetInstance().ProcessMatching(parking_slot_meas, matching_rs,
                                               mea_pose, type);

  std::cout << "Map Element Num: "
            << SemanticMap::GetInstance().GetMapLandmarkNum(
                   SEMANTIC_TYPE_PARKING_SLOT)
            << std::endl;

  std::cout << "Map Initialized Element Num: "
            << SemanticMap::GetInstance().GetMapInitializedLandmarkNum(
                   SEMANTIC_TYPE_PARKING_SLOT)
            << std::endl;
}

bool EkfEstimator::get_pose(const double ts, Pose &pose) {
  if (_dr_buf.empty()) {
    return false;
  }

  if (ts < _dr_buf.begin()->first) {
    std::cout << "get pose before dr_buf begin\n";
    return false;
  }

  // if (ts > _dr_buf.rbegin()->first) {
  //   std::cout << "get pose after dr_buf rbegin\n";
  //   return false;
  // }

  auto it1 = _dr_buf.lower_bound(ts);
  auto it0 = it1;
  --it0;

  if (it0 == _dr_buf.end()) {
    return false;
  }

  double ratio = (ts - it0->first) / (it1->first - it0->first);
  Eigen::Vector2d pose_translation = interpolate_translation(
      Eigen::Vector2d(it0->second.pose.x, it0->second.pose.y),
      Eigen::Vector2d(it1->second.pose.x, it1->second.pose.y), ratio);
  double pose_yaw =
      interpolate_angle(it0->second.pose.yaw, it1->second.pose.yaw, ratio);

  pose.x = pose_translation[0];
  pose.y = pose_translation[1];
  pose.yaw = pose_yaw;

  // std::cout << "---- " << t << " " << pose_translation.transpose() << " "
  //           << pose_yaw << std::endl;

  return true;
}

void EkfEstimator::sort_semantic_meas(
    const std::vector<SemanticMea::Ptr> &semantic_meas,
    std::unordered_map<SensorType, std::vector<SemanticMea::Ptr>>
        &sorted_meas) {
  for (size_t i = 0; i < semantic_meas.size(); ++i) {
    sorted_meas[semantic_meas.at(i)->GetSemanticMeaType()].push_back(
        semantic_meas.at(i));
  }

  for (auto it = sorted_meas.begin(); it != sorted_meas.end(); ++it) {
    const auto &type = it->first;
    it->second = MeaPreprocessor::GetInstance().Preprocess(type, it->second);
  }
}

void EkfEstimator::InputKinematicMea(
    const double ts, const std::vector<KinematicMea::Ptr> &kinetic_meas) {
  auto start = std::chrono::steady_clock::now();

  for (size_t i = 0; i < kinetic_meas.size(); ++i) {
    if (kinetic_meas.at(i)->GetKineticMeaType() == KINEMATIC_TYPE_ODO) {
      process_odo_mea(ts, kinetic_meas.at(i));
    }
  }

  auto end = std::chrono::steady_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
          .count();

  // std::cout << "Propagation Duration: " << duration << " MicroSeconds"
  // << std::endl;
}

bool EkfEstimator::Initialized() const {
  return EKFManagement::GetInstance().Initialized();
}

double EkfEstimator::interpolate_angle(const double angle0, const double angle1,
                                       const double t) {
  double diff = std::atan2(std::sin(angle1 - angle0),
                           std::cos(angle1 - angle0));  // shortest angle diff
  return angle0 + t * diff;
}

Eigen::Vector2d EkfEstimator::interpolate_translation(
    const Eigen::Vector2d &twb0, const Eigen::Vector2d &twb1, const double t) {
  return twb0 + t * (twb1 - twb0);
}

void EkfEstimator::process_odo_mea(const double ts,
                                   const KinematicMea::Ptr odo_mea) {
  const Eigen::VectorXd &odo_data = odo_mea->GetMeaData();
  double v = odo_data[0];
  double w = odo_data[1];

  {
    std::lock_guard<std::mutex> lock(_data_mutex);
    // std::cout << "=========== " << dt << " " << v << " " << w << std::endl;
    EKFManagement::GetInstance().Propagate(ts, v, w);
  }
  double latest_ts;
  Eigen::VectorXd latest_x;
  Eigen::MatrixXd latest_P;
  if (EKFManagement::GetInstance().GetLatestVechileState(latest_ts, latest_x,
                                                         latest_P)) {
    Pose dr_pose;
    dr_pose.x = latest_x[0];
    dr_pose.y = latest_x[1];
    dr_pose.yaw = latest_x[2];
    DrInfo dr_info;
    dr_info.pose = dr_pose;
    dr_info.angular_velocity = w;
    dr_info.velocity = v;
    _dr_buf.insert({ts, dr_info});
    std::cout << "Pose: " << dr_pose.x << " " << dr_pose.y << " " << dr_pose.yaw
              << std::endl;
    if (fabs(_dr_buf.begin()->first - _dr_buf.rbegin()->first) >
        ApaParameters::GetInstance().GetEstimatorParamters().buf_len) {
      _dr_buf.erase(_dr_buf.begin());
    }
  }
}
}  // namespace apa_slam
