/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/estimator.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Friday, May 9th 2025, 9:27:59 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "kinematic_measurement.h"
#include "parking_slot_tracker.h"
#include "semantic_measurement.h"
#include "tracker_base.h"

#include "semantic_map.h"

namespace apa_slam {
class EkfEstimator {
 public:
  typedef std::shared_ptr<EkfEstimator> Ptr;

  EkfEstimator();

  static EkfEstimator& GetInstance();

  void Init();

  void InputSemanticMea(const double ts,
                        const std::vector<SemanticMea::Ptr>& semantic_meas);

  void InputKinematicMea(const double ts,
                         const std::vector<KinematicMea::Ptr>& kinetic_meas);

  Pose GetLatestPose();

  bool Initialized() const;

  double GetLatestTimestamp();

 private:
  void process_odo_mea(const double ts, const KinematicMea::Ptr odo_mea);

  void sort_semantic_meas(
      const std::vector<SemanticMea::Ptr>& semantic_meas,
      std::unordered_map<SensorType, std::vector<SemanticMea::Ptr> >&
          sorted_meas);

  void process_parking_slot_meas(
      const double ts, const std::vector<SemanticMea::Ptr>& parking_slot_meas);

  bool get_pose(const double ts, Pose& pose);

  double interpolate_angle(const double angle0, const double angle1,
                           const double t);
  Eigen::Vector2d interpolate_translation(const Eigen::Vector2d& twb0,
                                          const Eigen::Vector2d& twb1,
                                          const double t);

  double _ts;
  Eigen::VectorXd _mean;
  Eigen::MatrixXd _cov;
  bool _initialized{false};
  std::mutex _data_mutex;

  std::unordered_map<SensorType, TrackerBase::Ptr> _tracker_pools;

  std::map<double, DrInfo> _dr_buf;
};
}  // namespace apa_slam
