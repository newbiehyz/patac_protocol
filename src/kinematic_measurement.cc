/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/src/kinematic_measurement.cc Path:
 * /home/yukan/Documents/work/local_mapping/src Created Date: Thursday, May 8th
 * 2025, 2:36:40 pm Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "kinematic_measurement.h"
namespace apa_slam {

KinematicMea::KinematicMea(const SensorType &type, const double timestamp)
    : _type(type), _timestamp(timestamp) {}

SensorType KinematicMea::GetKineticMeaType() { return _type; }

double KinematicMea::GetMeaTimestamp() { return _timestamp; }

}  // namespace apa_slam
