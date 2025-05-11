/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/semantic_map.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Thursday, May 8th 2025, 3:55:53 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once

#include <memory>
#include <unordered_map>
#include <mutex>
#include "local_mapping_define.h"
#include "parking_slot_landmark.h"
#include "semantic_landmark.h"
namespace apa_slam {
class SemanticMap {
 public:
  typedef std::shared_ptr<SemanticMap> Ptr;
  SemanticMap();
  static SemanticMap& GetInstance();

  void AddLandmark(const SensorType type, SemanticLandmark::Ptr landmark);
  bool HasMap(const SensorType type) const;
  const std::unordered_map<int, SemanticLandmark::Ptr>& GetMap(
      const SensorType type);
  void ClearMap();
  int GetMapLandmarkNum(const SensorType& type);
  bool HasLandmark(const SensorType type, const int id);

 private:
  std::unordered_map<SensorType, std::unordered_map<int, SemanticLandmark::Ptr>>
      _map;

  std::mutex _data_mutex;
};
}  // namespace apa_slam
