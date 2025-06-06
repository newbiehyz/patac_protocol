/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/semantic_map.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Thursday, May 8th 2025, 3:55:53 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <unordered_map>

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
  const std::map<int, SemanticLandmark::Ptr>& GetMap(const SensorType type);
  void ClearMap();
  int GetMapLandmarkNum(const SensorType& type);
  int GetMapInitializedLandmarkNum(const SensorType& type);
  bool HasLandmark(const SensorType type, const int id);
  void AddMea(const SensorType type, const int id, const SemanticMea::Ptr mea,
              const Pose& mea_pose);

  void InitializeLandmark(const SensorType type, const int id,
                          const Eigen::VectorXd& vehicle_mean,
                          Eigen::MatrixXd& vehicle_P, Eigen::MatrixXd& Jx);

  void InitializeLandmark(const SensorType type, const int id, const long long timestamp,
                          const Eigen::VectorXd& vehicle_mean,
                          Eigen::MatrixXd& vehicle_P, Eigen::MatrixXd& Jx);

  void GetEKFDataList(
      std::map<SensorType, std::set<int>>& augmentation_list,
      std::map<SensorType, std::set<int>>& update_list,
      std::map<SensorType, std::set<int>>& marginalization_list);

  void GetSlidingWindowEKFDataList(
      const std::vector<long long>& slw_timestamp,
      std::vector<std::map<SensorType, std::vector<int>>>& sw_lm_list,
      std::map<SensorType, std::set<int>>& marginalization_list);

  Eigen::MatrixXd GetLandmarkCov(const SensorType& type, const int id);

  void SetLandmarkCov(const SensorType& type, const int id,
                      const Eigen::MatrixXd& cov);

  void SetLandmarkMean(const SensorType& type, const int id,
                       const Eigen::VectorXd& mean);

  const SemanticLandmark::Ptr GetLandmark(const SensorType& type,
                                          const int& id);

  void TagMarginalization(const long long timestamp);

  void MarginLandmark(const SensorType& type, const int& id);

  bool GetFullLocalMap(const SensorType& type, const Pose& pose,
                       std::vector<SemanticLandmark::Ptr>& local_map);

 private:
  std::map<SensorType, std::map<int, SemanticLandmark::Ptr>> _map;

  std::mutex _data_mutex;
};
}  // namespace apa_slam
