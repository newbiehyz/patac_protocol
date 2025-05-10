/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/src/semantic_measurement.cc Path:
 * /home/yukan/Documents/work/local_mapping/src Created Date: Thursday, May 8th
 * 2025, 1:54:57 pm Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include "semantic_measurement.h"

namespace apa_slam {
SemanticMea::SemanticMea(const SensorType& type, const double timestamp)
    : _type(type), _timestamp(timestamp) {}

SensorType SemanticMea::GetSemanticMeaType() { return _type; }

}  // namespace apa_slam
