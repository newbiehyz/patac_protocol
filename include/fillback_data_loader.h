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
#include <nlohmann/json.hpp>
#include <set>

#include "kinematic_measurement.h"
#include "local_mapping_define.h"
#include "odo_measurement.h"
#include "parking_slot_measurement.h"
#include "semantic_measurement.h"
using json = nlohmann::json;
#define WHEEL_BASE 3.16
#define R_N_557_ROW 5
#define R_N_557_COL 15

#define INVALID_VALUE 99999999
#define VEHICLE_LENGTH 5259.9
#define REAR_AXLE_CENTER_VEHICLE_REAR 1136.7
#define BIRD_VIEW_HEIGHT 896                                       // 896
#define BIRD_VIEW_WIDTH 896                                        // 896
#define REAR_BIRD_VIEW_HEIGHT 896                                  // 896
#define REAR_BIRD_VIEW_WIDTH 896                                   // 896
#define LR_BIRD_PIXECL_2_WORLD (20000.0 / BIRD_VIEW_HEIGHT)        // 20000.0
#define REAR_BIRD_PIXECL_2_WORLD (20000.0 / REAR_BIRD_VIEW_WIDTH)  // 20000.0

static float RADIUS_557_TAB[R_N_557_ROW][R_N_557_COL] = {
    {540, 510, 480, 450, 420, 390, 360, 330, 300, 270, 240, 210, 180, 150, 120},
    // LF
    {4.712f, 5.175f, 5.596f, 6.084f, 6.658f, 7.324f, 8.049f, 8.867f, 9.855f,
     11.052f, 12.523f, 14.445f, 16.949f, 20.488f, 25.616f},
    // RF
    {4.732f, 5.136f, 5.571f, 6.094f, 6.668f, 7.314f, 8.048f, 8.901f, 9.887f,
     11.087f, 12.579f, 14.494f, 17.033f, 20.590f, 25.865f},
    // LR
    {4.748f, 5.186f, 5.651f, 6.161f, 6.722f, 7.351f, 8.072f, 8.908f, 9.910f,
     11.089f, 12.569f, 14.400f, 16.857f, 20.273f, 25.568f},
    // RR
    {4.750f, 5.186f, 5.633f, 6.147f, 6.707f, 7.337f, 8.057f, 8.904f, 9.894f,
     11.080f, 12.586f, 14.450f, 16.947f, 20.419f, 25.585f}};

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
  void load_odo_meas(const std::string& odo_mea_file);
  void load_pose_odo_meas(const std::string& pose_file,
                          const std::string& can_file);
  void load_pose_odo_meas2(const std::string& pose_file);
  double get_angular_velocity(const double velocity,
                              const double steering_angle, const int gear);

  void load_semantic_meas(const std::string& semantic_mea_file);
  double angle_diff(double angle1, double angle2);
  Eigen::VectorXd interpolate_pose(
      const long long timestamp,
      const std::map<long long, Eigen::Vector3d>& pose_data);
  std::map<long long, std::vector<SemanticMea::Ptr>> _semantic_mea;
  std::map<long long, std::vector<KinematicMea::Ptr>> _kinematic_mea;
  std::map<long long, std::vector<std::pair<ReplaySensorType, long long>>> _mea_seq;
  std::map<long long, std::vector<std::pair<ReplaySensorType, long long>>>::iterator
      _mea_it;

  std::map<long long, Eigen::Vector3d> _pose_data;
  int _mea_id{0};
};
}  // namespace apa_slam
