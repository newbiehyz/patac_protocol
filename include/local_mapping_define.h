/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/local_mapping_define.h Path:
 * /home/yukan/Documents/work/local_mapping/include Created Date: Thursday, May
 * 8th 2025, 1:36:50 pm Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#pragma once

#include <Eigen/Eigen>
#include <cstdint>
#include <mutex>
#include <set>
#include <shared_mutex>

#include "cross_correlation_id.h"
// x0 x1 x2 x3
// y0 y1 y2 y3
#define DATA_COLS_PARKING_SLOT 2
#define DATA_ROWS_PARKING_SLOT 2
#define DATA_ROWS_ODO 2

#define STATE_VEHICLE_SIZE 3
#define STATE_PARKING_SLOT_SIZE 4

#define RESIDUAL_PARKING_SLOT_SIZE 4

#define NOISE_ODO_SIZE 2
#define NOISE_PARKING_SLOT 4

#define VEHICLE_WIDTH 2.0
#define VEHICLE_HEIGHT 1.5
#define VEHICLE_FORWARD_DIST 4.3
#define VEHICLE_BACKWARD_DIST -0.96

#define BUMPER_ALERT_RANGE 3.0

namespace apa_slam {

struct SimulationParams {
  double slot_mea_noise_x;
  double slot_mea_noise_y;
  double odo_velocity_noise;
  double odo_angular_velocity_noise;
  double perception_sensing_range;
};

struct DatasetParams {
  long long min_dataset_timestamp;
  long long max_dataset_timestamp;
  double timedelay;
  bool use_udp;
  std::string udp_ip;
  int udp_port;
};

enum ParkingSlotType { Vertical = 0, Horizontal = 1, Oblique = 2};

enum Action : std::uint8_t { RESET = 0, MAPPING = 1, SAVE = 2}; // SAVE action reserved

struct ParkingSlotAttribute {
  bool parkable;
  ParkingSlotType slot_type;
};
struct EstimatorParams {
  double slot_mea_noise_x;
  double slot_mea_noise_y;
  double odo_velocity_noise;
  double odo_angular_velocity_noise;
  double slot_matching_dist_thresh;
  double slot_matching_angle_thresh;
  double duplicate_slot_thresh;
  double slot_local_map_range;
  int slot_min_tracking_times;
  double margin_tracking_time;
  bool export_debug_file;
  std::string debug_file_path;
  bool use_time_compensate;
  double buf_len;
  double vertical_slot_len;
  double horizontal_slot_len;
  double slot_mea_max_range;
  double time_scale;
  int window_size;
  double sl_translation_th;
  double sl_angle_th;
  double slot_inward_tunning;
  bool use_loc_convert;
  int slot_confirm_time_sec;
};

enum ReplaySensorType { REPLAY_TYPE_SEMANTIC = 0, REPLAY_TYPE_KINEMATIC = 1 };

enum SensorType : std::uint8_t {
  KINEMATIC_TYPE_ODO = 0,
  KINEMATIC_TYPE_IMU = 1,
  SEMANTIC_TYPE_PARKING_SLOT = 2,
  SEMANTIC_TYPE_PARKING_COLUMN = 3,
  DEFAULT = 4
};

struct Pose {
  double x;
  double y;
  double yaw;
};

struct DrInfo {
  Pose pose;
  double velocity;
  double angular_velocity;
};

struct UdpData {
  float pose[3];
  float timestamp;
  float slot_corners[320];
  int8_t slot_num;
};

struct FilterInfo {
  std::unordered_map<CrossCorrelationKey, Eigen::MatrixXd>
      lm_cross_correlation;  // P_a_b  a=first b=second
  std::unordered_map<CrossCorrelationId, Eigen::MatrixXd>
      state_lm_cross_correlation;  // P_state_lm

  Eigen::MatrixXd N;  // odo measurement

  std::unordered_map<SensorType, std::set<int>> lm_state_list;  // full state

  Eigen::VectorXd vehicle_x;
  Eigen::MatrixXd vehicle_P;
  double vehicle_v;
  double vehicle_w;
  double ts;
};

struct  VisualizationMeas {
  std::mutex meas_mutex;
  std::vector<Eigen::VectorXd> slot_meas;
  bool saveMap{false};
  bool startMapping{false};
  bool IsLoadMap{false};
  bool startLocalization{false};
  std::string slot_map_data_filename{"marginalization_data.bin"};
};

extern VisualizationMeas vis_meas;

struct SlwVisualization {
  std::shared_mutex mutex;

  std::vector<Eigen::VectorXd> sl_pose;
  std::vector<std::vector<Eigen::VectorXd>> sl_meas;
};

extern SlwVisualization gl_slw;

extern bool only_localization;

}  // namespace apa_slam
