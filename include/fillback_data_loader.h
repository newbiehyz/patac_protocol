/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/fillback_data_loader.h Path:
 * /home/yukan/Documents/work/local_mapping/include Created Date: Monday, May
 * 19th 2025, 9:22:19 am Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>

#include "kinematic_measurement.h"
#include "local_mapping_define.h"
#include "math_utils.h"
#include "nlohmann/json.hpp"
#include "odo_measurement.h"
#include "parking_slot_measurement.h"
#include "semantic_measurement.h"

using json = nlohmann::json;
#define WHEEL_BASE 3.16
#define R_N_557_ROW 5
#define R_N_557_COL 15

#define APA_INVALID_VALUE 99999999
#define APA_VEHICLE_LENGTH 5259.9
#define APA_REAR_AXLE_CENTER_VEHICLE_REAR 1136.7
#define APA_BIRD_VIEW_HEIGHT 896                                     // 896
#define APA_BIRD_VIEW_WIDTH 896                                      // 896
#define APA_REAR_BIRD_VIEW_HEIGHT 896                                // 896
#define APA_REAR_BIRD_VIEW_WIDTH 896                                 // 896
#define APA_LR_BIRD_PIXECL_2_WORLD (20000.0 / APA_BIRD_VIEW_HEIGHT)  // 20000.0
#define APA_REAR_BIRD_PIXECL_2_WORLD \
  (20000.0 / APA_REAR_BIRD_VIEW_WIDTH)  // 20000.0

namespace apa_slam {
class FillbackDataLoader {
 public:
  typedef std::shared_ptr<FillbackDataLoader> Ptr;
  FillbackDataLoader();
  void LoadDataSet(const std::string& dataset_path);
  bool PopOutMea(ReplaySensorType& type, long long& arriving_ts,
                 long long& sensor_ts);
  std::vector<SemanticMea::Ptr> GetSemanticMeas(const long long timestamp);
  std::vector<KinematicMea::Ptr> GetKinematicMeas(const long long timestamp);
  Eigen::Vector2d ConvertUvToVehicle(const Eigen::Vector2d& uv);

  static FillbackDataLoader& GetInstance();

 private:
  void load_pose_odo_meas(const std::string& pose_file,
                          const std::string& can_file);
  void load_pose_odo_meas2(const std::string& pose_file);

  void load_semantic_meas(const std::string& semantic_mea_file);
  double angle_diff(double angle1, double angle2);
  Eigen::VectorXd interpolate_pose(
      const long long timestamp,
      const std::map<long long, Eigen::Vector3d>& pose_data);
  std::map<long long, std::vector<SemanticMea::Ptr>> _semantic_mea;
  std::map<long long, std::vector<KinematicMea::Ptr>> _kinematic_mea;
  std::map<long long, std::vector<std::pair<ReplaySensorType, long long>>>
      _mea_seq;
  std::map<long long,
           std::vector<std::pair<ReplaySensorType, long long>>>::iterator
      _mea_it;

  std::map<long long, Eigen::Vector3d> _pose_data;
  int _mea_id{0};
};
}  // namespace apa_slam
