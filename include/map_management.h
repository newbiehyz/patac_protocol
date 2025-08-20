/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/map_manager.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Saturday, May 10th 2025, 4:11:12 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once
#include <memory>

#include "semantic_map.h"
#include "semantic_measurement.h"
#include "map_io.h"

namespace apa_slam {
class MapManagement {
 public:
  typedef std::shared_ptr<MapManagement> Ptr;
  MapManagement();
  static MapManagement& GetInstance();
  void ProcessMatching(const std::vector<SemanticMea::Ptr>& meas,
                       const std::vector<int>& map_matching,
                       const Pose& mea_pose, const SensorType& type);

 private:
  void process_slot_matching(const std::vector<SemanticMea::Ptr>& meas,
                             const std::vector<int>& map_matching,
                             const Pose& mea_pose);
};
}  // namespace apa_slam
