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

// x0 x1 x2 x3
// y0 y1 y2 y3
#define DATA_COLS_PARKING_SLOT 2
#define DATA_ROWS_PARKING_SLOT 2
#define DATA_ROWS_ODO 2

#define STATE_VEHICLE_SIZE 3
#define STATE_PARKING_SLOT_SIZE 4



#define VEHICLE_WIDTH 1.0
#define VEHICLE_HEIGHT 1.0
#define VEHICLE_FORWARD_DIST 2.518
#define VEHICLE_BACKWARD_DIST -0.921

#define BUMPER_ALERT_RANGE 3.0

namespace apa_slam {

struct SimulationParams {
  double slot_mea_noise_x;
  double slot_mea_noise_y;
  double odo_velocity_noise;
  double odo_angular_velocity_noise;
  double perception_sensing_range;
};

struct EstimatorParams {
  double slot_matching_dist_thresh;
  double slot_matching_angle_thresh;
};

enum ReplaySensorType { REPLAY_TYPE_SEMANTIC = 0, REPLAY_TYPE_KINEMATIC = 1 };

enum SensorType {
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

}  // namespace apa_slam
