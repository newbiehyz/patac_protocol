/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/visualization/pangolin_drawer.h
 * Path: /home/yukan/Documents/work/local_mapping/include/visualization
 * Created Date: Friday, May 9th 2025, 11:53:30 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once

#include <pangolin/pangolin.h>

#include <random>

#include "ekf_estimator.h"

namespace apa_slam {
static std::string font_ttf = "../Pangolin-Regular.ttf";
class PangolinDrawer {
 public:
  typedef std::shared_ptr<PangolinDrawer> Ptr;
  PangolinDrawer();

  void DrawAPA();

 private:
  void draw_local_map(const Pose& pose);

  void draw_vehicle(const Pose& latest_pose, const Eigen::MatrixXd& latest_cov);
  void draw_vehicle_bbox();
  void draw_parking_slot(const int& id, const Eigen::MatrixXd& data);
  void draw_traj();
  void draw_local_meas();
  void draw_sliding_window();

  std::map<long long, Pose> _traj;

  std::shared_ptr<pangolin::GlFont> _font;

  std::vector<Eigen::Vector3d> _sl_color;
};
}  // namespace apa_slam
