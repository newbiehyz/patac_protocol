/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/math_utils.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Friday, May 9th 2025, 9:28:19 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

 #pragma once
#include <Eigen/Eigen>

namespace apa_slam {
inline double AngleDiff(double angle1, double angle2) {
  double diff = angle1 - angle2;
  while (diff > M_PI)
    diff -= 2 * M_PI;
  while (diff < -M_PI)
    diff += 2 * M_PI;
  return diff;
}
} // namespace apa_slam
