/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/tracker_base.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Friday, May 9th 2025, 3:39:18 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "tracker_base.h"

namespace apa_slam {
TrackerBase::TrackerBase(const SensorType type) : _type(type) {}

SensorType TrackerBase::GetTrackerType() { return _type; }

}  // namespace apa_slam
