/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/tracker_base.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Friday, May 9th 2025, 3:38:46 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once

#include <iostream>
#include <memory>

#include "local_mapping_define.h"
#include "semantic_measurement.h"

namespace apa_slam {
class TrackerBase {
 public:
  typedef std::shared_ptr<TrackerBase> Ptr;
  TrackerBase(const SensorType type);
  SensorType GetTrackerType();
  virtual ~TrackerBase() = default;
  virtual std::vector<int> HungarianMatching(
      const std::vector<SemanticMea::Ptr>& meas, const Pose& pose) = 0;

 protected:
  std::vector<SemanticMea::Ptr> _last_meas;
  std::vector<int> _last_map_matching;
  Pose _last_pose;

 private:
  SensorType _type;
  double _timestamp;
};

}  // namespace apa_slam
