/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/visualization/pangolin_drawer.h
 * Path: /home/yukan/Documents/work/local_mapping/include/visualization
 * Created Date: Friday, May 9th 2025, 11:53:30 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include <ekf_estimator.h>
#include <pangolin/pangolin.h>

#include <queue>
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

  std::map<double, Pose> _traj;

  std::shared_ptr<pangolin::GlFont> _font;
};
}  // namespace apa_slam
