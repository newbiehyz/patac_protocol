/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/sliding_window.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Friday, May 30th 2025, 5:09:23 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once
#include <Eigen/Eigen>
#include <deque>
#include <memory>

#include "apa_parameters.h"

namespace apa_slam {
class SlidingWindow {
 public:
  typedef std::shared_ptr<SlidingWindow> Ptr;
  SlidingWindow();
  void Init();
  bool Initialized();

  void AddFrame(const long long ts, const Eigen::VectorXd &pose);

 private:
  bool _inited{false};

  std::vector<long long> _sl_timestamp;
  std::vector<Pose> _sl_pose;
};
}  // namespace apa_slam
