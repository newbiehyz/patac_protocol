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
  _mean = Eigen::VectorXd::Zero(3);
  _cov = 1e-7 * Eigen::MatrixXd::Identity(3, 3);

  TrackerBase::Ptr parking_slot_tracker =
      std::make_shared<ParkingSlotTracker>();
  _tracker_pools.insert({SEMANTIC_TYPE_PARKING_SLOT, parking_slot_tracker});

  SemanticMap::GetInstance().ClearMap();
}

double EkfEstimator::GetLatestTimestamp() {
  std::lock_guard<std::mutex> lock(_data_mutex);
  return _ts;
}

Pose EkfEstimator::GetLatestPose() {
  std::lock_guard<std::mutex> lock(_data_mutex);
  Pose latest_pose;
  latest_pose.x = _mean[0];
  latest_pose.y = _mean[1];
  latest_pose.yaw = _mean[2];

  return latest_pose;
}

EkfEstimator &EkfEstimator::GetInstance() {
  static EkfEstimator instance;
  return instance;
}

void EkfEstimator::InputSemanticMea(
    const double ts, const std::vector<SemanticMea::Ptr> &semantic_meas) {
  std::unordered_map<SensorType, std::vector<SemanticMea::Ptr> > meas_sorted;
  sort_semantic_meas(semantic_meas, meas_sorted);

  for (auto it = meas_sorted.begin(); it != meas_sorted.end(); ++it) {
    const SensorType mea_type = it->first;
    process_semantic_meas(mea_type, ts, it->second);
  }
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

  MapManagement::GetInstance().ProcessMatching(parking_slot_meas, matching_rs,
                                               mea_pose, type);

  std::cout << "Map Size: " << SemanticMap::GetInstance().GetMapLandmarkNum(SEMANTIC_TYPE_PARKING_SLOT) << std::endl;
}

bool EkfEstimator::get_pose(const double ts, Pose &pose) {
  if (_dr_buf.empty()) {
    return false;
  }

  if (ts < _dr_buf.begin()->first) {
    std::cout << "get pose before dr_buf begin\n";
    return false;
  }

  if (ts > _dr_buf.rbegin()->first) {
    std::cout << "get pose after dr_buf rbegin\n";
    return false;
  }

  auto it1 = _dr_buf.lower_bound(ts);
  auto it0 = it1;
  --it0;

  if (it0 == _dr_buf.end()) {
    return false;
  }

  double t = (ts - it0->first) / (it1->first - it0->first);
  Eigen::Vector2d pose_translation = interpolate_translation(
      Eigen::Vector2d(it0->second.pose.x, it0->second.pose.y),
      Eigen::Vector2d(it1->second.pose.x, it1->second.pose.y), t);
  double pose_yaw =
      interpolate_angle(it0->second.pose.yaw, it1->second.pose.yaw, t);

  pose.x = pose_translation[0];
  pose.y = pose_translation[1];
  pose.yaw = pose_yaw;

  // std::cout << "---- " << t << " " << pose_translation.transpose() << " "
  //           << pose_yaw << std::endl;

  return true;
}

void EkfEstimator::sort_semantic_meas(
    const std::vector<SemanticMea::Ptr> &semantic_meas,
    std::unordered_map<SensorType, std::vector<SemanticMea::Ptr> >
        &sorted_meas) {
  for (size_t i = 0; i < semantic_meas.size(); ++i) {
    sorted_meas[semantic_meas.at(i)->GetSemanticMeaType()].push_back(
        semantic_meas.at(i));
  }
}

void EkfEstimator::InputKinematicMea(
    const double ts, const std::vector<KinematicMea::Ptr> &kinetic_meas) {
  for (size_t i = 0; i < kinetic_meas.size(); ++i) {
    if (kinetic_meas.at(i)->GetKineticMeaType() == KINEMATIC_TYPE_ODO) {
      process_odo_mea(ts, kinetic_meas.at(i));
    }
  }
}

bool EkfEstimator::Initialized() const { return _initialized; }

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
  if (!_initialized) {
    _ts = ts;
    _initialized = true;
    return;
  }
  const Eigen::VectorXd &odo_data = odo_mea->GetMeaData();
  double v = odo_data[0];
  double w = odo_data[1];

  Eigen::Vector2d twb;
  double yaw;
  {
    std::lock_guard<std::mutex> lock(_data_mutex);
    twb = _mean.head(2);
    yaw = _mean[2];
  }

  Eigen::Rotation2Dd rot(yaw);
  Eigen::Matrix2d Rwb = rot.toRotationMatrix();
  Eigen::Vector2d dir = Rwb.col(0);
  double dt = ts - _ts;
  twb += dir * v * dt;
  yaw += dt * w;
  {
    std::lock_guard<std::mutex> lock(_data_mutex);
    _mean.head(2) = twb;
    _mean[2] = yaw;
  }

  Pose dr_pose;
  dr_pose.x = twb.x();
  dr_pose.y = twb.y();
  dr_pose.yaw = yaw;
  DrInfo dr_info;
  dr_info.pose = dr_pose;
  dr_info.angular_velocity = w;
  dr_info.velocity = v;
  _dr_buf.insert({ts, dr_info});
  _ts = ts;
}
}  // namespace apa_slam
